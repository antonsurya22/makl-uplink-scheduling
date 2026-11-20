#!/usr/bin/env python3
"""E7 uplink links for the live bridge (protocol E7_PROTOCOL_FREEZE.md, section 3).

K0 = the frozen GATE_B1 SharedLink itself (shared FIFO, no expiry for Q2 best-effort traffic = E5 F-N2).
K1-K3 = the E4 DisciplineLink (e4_frozen/link_disciplines.py, byte-identical to the E4 v3 freeze), IMPORTED:
  K1 'fifo'     + camera expiry 1 s (scans and probe ACKs never expire)
  K2 'prio'     strict priority: scan and probe ACK before camera; no expiry
  K3 'keeplast' a new message replaces its stream's older messages that have not started transmission; no expiry
Both link classes are SUBCLASSED only to (a) write a per-message ledger and (b) give the bridge the attributes it reads
(`records[mid].finished_us`, `outcomes()`, `tx_log`). Queueing, service order, expiry and timing are the parent code.
One mapping: the bridge (B5/C2 _flush) knows the terminal statuses delivered / queue_drop / channel_loss / expired only.
A DisciplineLink record that ends 'replaced' (K3) is therefore shown to the bridge as 'queue_drop' (so its cache entry
is released); the ledger keeps the true status 'replaced'. The scheduler only tests status == 'queued', so this
cannot change service order or timing (checked by tests/test_e7_offline.py against the E4 replay)."""
import json,sys
from collections import Counter,deque
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'e4_frozen'))
import link_disciplines as LD
from shared_link import SharedLink            # frozen GATE_B1 code (SHA256 04f29070...), already on sys.path in the bridge
CONFIGS={'K0':{'link':'shared_link','discipline':'fifo','expiry_us':{}},
         'K1':{'link':'discipline','discipline':'fifo','expiry_us':{'camera':1_000_000}},
         'K2':{'link':'discipline','discipline':'prio','expiry_us':{}},
         'K3':{'link':'discipline','discipline':'keeplast','expiry_us':{}}}
TERMINAL=('delivered','queue_drop','channel_loss','expired')
class Ledger:
    """One JSON line per message when it reaches a terminal status; header line first. path=None keeps lines in memory."""
    def __init__(self,path,header):
        self.lines=[];self.f=open(path,'a',buffering=1) if path else None;self.write(dict(header,header=True))
    def write(self,obj):
        s=json.dumps(obj,separators=(',',':'))
        if self.f:self.f.write(s+'\n')
        else:self.lines.append(obj)
class _LedgerMixin:
    def _ledger_init(self,ledger):
        self.ledger=ledger;self._pend={};self.reliable_offers=0
    def _offer_note(self,mid,stream,payload,cam_q,rel):
        self._pend[mid]={'mid':mid,'stream':stream,'bytes':payload,'offer_us':self.now_us,'cap_bps':self.capacity_bps,'rel':rel,
                         'cam_q_bytes':cam_q,'q_bytes':self.waiting_bytes,'tx_us':[]}
    def _tx_note(self,mid,start_us):
        p=self._pend.get(mid)
        if p is not None:p['tx_us'].append(start_us)
    def _finish_note(self,mid,status):
        p=self._pend.pop(mid,None)
        if p is not None and self.ledger is not None:self.ledger.write(dict(p,finish_us=self.now_us,status=status))
class LedgerSharedLink(_LedgerMixin,SharedLink):
    """K0: the frozen SharedLink, plus the ledger. Behaviour is the parent's (equality test in tests/)."""
    def __init__(self,*,ledger=None,**kw):
        super().__init__(**kw);self._ledger_init(ledger)
    def _cam_queued(self):
        return sum(p.wire_bytes for p in self.waiting if self.records[p.message_id].stream=='camera' and self.records[p.message_id].status=='queued')
    def offer(self,*,stream,payload_bytes,stamp_us,reliability='best_effort'):
        if reliability!='best_effort':self.reliable_offers+=1
        self._offer_note(self.next_message_id+1,stream,payload_bytes,self._cam_queued(),reliability!='best_effort')
        return super().offer(stream=stream,payload_bytes=payload_bytes,stamp_us=stamp_us,reliability=reliability)
    def _schedule(self,when_us,event,data):
        if event=='tx_done':self._tx_note(data[0].message_id,data[1])
        super()._schedule(when_us,event,data)
    def _finish(self,record,status):
        before=record.status;super()._finish(record,status)
        if before not in TERMINAL and record.status==status:self._finish_note(record.message_id,status)
class RecView(LD.Rec):
    """Same slots as E4 Rec; adds the read-only names the bridge uses."""
    __slots__=()
    @property
    def finished_us(self):return self.finished
    @property
    def message_id(self):return self.mid
    @property
    def payload_bytes(self):return self.payload
class QdiscLink(_LedgerMixin,LD.DisciplineLink):
    """K1-K3: the E4 DisciplineLink, plus the ledger, the bridge-facing names and a prune for the bridge's 30 s cleanup."""
    def __init__(self,*,ledger=None,**kw):
        super().__init__(**kw);self._ledger_init(ledger);self.tx_log=deque(maxlen=1);self.replaced=Counter()
    def _queued_entries(self):
        if self.disc in ('fifo','keeplast'):return list(self.fifo)
        return [pk for d in self.q.values() for pk in d]
    def _cam_queued(self):
        return sum(w for mid,i,w in self._queued_entries() if self.records[mid].stream=='camera' and self.records[mid].status=='queued')
    def offer(self,*,stream,payload_bytes,stamp_us,reliability='best_effort',expiry_us='stream'):
        if reliability!='best_effort':self.reliable_offers+=1      # E7 runs Q2 only; counted and reported, expiry stays per config
        nid=self.next_id+1;self._offer_note(nid,stream,payload_bytes,self._cam_queued(),reliability!='best_effort')
        mid=super().offer(stream=stream,payload_bytes=payload_bytes,stamp_us=stamp_us,expiry_us=expiry_us)
        if mid!=nid:raise RuntimeError('E7_LEDGER_ID_MISMATCH')
        self.records[mid].__class__=RecView
        return mid
    def _sched(self,t,ev,data):
        if ev=='tx_done':self._tx_note(data[0],self.now_us)
        super()._sched(t,ev,data)
    def _finish(self,r,st):
        if r.status!='queued':return
        super()._finish(r,st);self._finish_note(r.mid,st)
        if st=='replaced':r.status='queue_drop';self.replaced[r.stream]+=1   # bridge-facing status; ledger keeps 'replaced'
    def outcomes(self):
        return dict(Counter(r.status for r in self.records.values()))
    def prune(self,keep):
        """Delete terminal records that no queue entry, event or bridge cache still references (as B5 does for B1)."""
        live={pk[0] for pk in self._queued_entries()}
        for _,_,ev,data in self.events:live.add(data[0] if ev=='tx_done' else data)
        for mid,r in list(self.records.items()):
            if r.status!='queued' and mid not in live and mid not in keep:del self.records[mid]
def make_link(config,*,capacity_bps,queue_limit_bytes,delay_us,seed=771,jitter_us=0,packet_loss=0.0,ledger_path=None,name='uplink'):
    """Same constructor values as B5 ContinuousBridge. Jitter and loss must be 0 (E3-B/E5 bridge: --jitter-ms 0, --loss 0)."""
    if config not in CONFIGS:raise ValueError('E7_UNKNOWN_CONFIG '+str(config))
    if jitter_us!=0 or packet_loss!=0:raise RuntimeError('E7_REQUIRES_NO_JITTER_NO_LOSS')
    c=CONFIGS[config]
    header={'config':config,'link':c['link'],'discipline':c['discipline'],'expiry_us':c['expiry_us'],
            'capacity_bps':capacity_bps,'queue_limit_bytes':queue_limit_bytes,'delay_us':delay_us}
    led=Ledger(ledger_path,header)
    if c['link']=='shared_link':
        L=LedgerSharedLink(ledger=led,name=name,seed=seed,capacity_bps=capacity_bps,queue_limit_bytes=queue_limit_bytes,
                           delay_us=delay_us,jitter_us=jitter_us,packet_loss=packet_loss)
        L.tx_log=deque(maxlen=1024)
    else:
        L=QdiscLink(ledger=led,capacity_bps=capacity_bps,queue_limit_bytes=queue_limit_bytes,delay_us=delay_us,
                    discipline=c['discipline'],expiry_us=c['expiry_us'])
    L.e7_config=config
    return L
