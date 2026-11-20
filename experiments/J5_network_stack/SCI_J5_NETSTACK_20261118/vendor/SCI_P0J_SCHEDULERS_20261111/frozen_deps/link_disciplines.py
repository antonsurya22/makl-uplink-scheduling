"""E4: generalized shared-uplink model with selectable queue discipline and per-stream expiry.

Same physics as the frozen GATE_B1 SharedLink (integer microseconds; MTU 1472 B payload per packet + 28 B overhead;
serial serializer at capacity_bps with ceil rounding; fixed one-way delay; atomic tail-drop admission against
queue_limit_bytes; no loss in E4, so no repairs). Differences are ONLY:
  * per-stream expiry (deadline = admission time + expiry_us[stream]; None = never expires);
  * discipline: 'fifo' (one shared FIFO, as frozen), 'prio' (strict priority: scan and probe ACK before camera),
    'drr' (per-stream deficit round robin, quantum 1500 B, equal weights: fq_codel-like flow isolation, no CoDel),
    'keeplast' (one shared FIFO, but a new message of a stream replaces that stream's older messages that have not
    started transmission: DDS keep-last-1-like).
In 'fifo' mode with the frozen expiry pattern this class reproduces the frozen SharedLink record by record
(checked by test_e4_equivalence.py).
"""
from __future__ import annotations
from collections import deque, Counter
import heapq, math

class Rec:
    __slots__=('mid','stream','payload','created','status','finished','total','got','deadline','started')
    def __init__(s,mid,stream,payload,created,total,deadline):
        s.mid=mid;s.stream=stream;s.payload=payload;s.created=created;s.status='queued';s.finished=None
        s.total=total;s.got=0;s.deadline=deadline;s.started=False

class DisciplineLink:
    HIGH=('scan','c52_probe_ack')
    def __init__(self,*,capacity_bps,queue_limit_bytes,delay_us,discipline='fifo',expiry_us=None,mtu=1472,overhead=28,quantum=1500):
        if discipline not in ('fifo','prio','drr','keeplast'):raise ValueError(discipline)
        self.capacity_bps=capacity_bps;self.qlim=queue_limit_bytes;self.delay=delay_us;self.disc=discipline
        self.expiry=dict(expiry_us or {});self.mtu=mtu;self.ovh=overhead;self.quantum=quantum
        self.now_us=0;self.events=[];self.serial=0;self.next_id=0;self.busy=False
        self.records={};self.waiting_bytes=0;self.wire=Counter()
        self.q={}            # stream -> deque of (mid, idx, wire)
        self.order=deque()   # for fifo/keeplast: global order of (stream) entries mirrors packets
        self.fifo=deque()
        self.deficit=Counter();self.rr=deque()
        self.bytes_by=Counter()   # (stream,status) -> payload bytes
    # ---------- helpers ----------
    def _sched(self,t,ev,data):
        if t<self.now_us:raise RuntimeError('past')
        self.serial+=1;heapq.heappush(self.events,(t,self.serial,ev,data))
    def _packets(self,mid,size):
        size=max(1,size);return [(mid,i,min(self.mtu,size-o)+self.ovh) for i,o in enumerate(range(0,size,self.mtu))]
    def _finish(self,r,st):
        if r.status!='queued':return
        r.status=st;r.finished=self.now_us;self.bytes_by[(r.stream,st)]+=r.payload
    def _queues_empty(self):
        if self.disc in ('fifo','keeplast'):return not self.fifo
        return not any(self.q.get(s) for s in self.q)
    def _push(self,stream,pk):
        if self.disc in ('fifo','keeplast'):self.fifo.append(pk)
        else:
            self.q.setdefault(stream,deque()).append(pk)
            if self.disc=='drr' and stream not in self.rr:self.rr.append(stream)
    def _pop(self):
        """Next packet to serve, or None."""
        if self.disc in ('fifo','keeplast'):return self.fifo.popleft() if self.fifo else None
        if self.disc=='prio':
            for s in list(self.HIGH)+[s for s in self.q if s not in self.HIGH]:
                if self.q.get(s):return self.q[s].popleft()
            return None
        # drr
        while self.rr:
            s=self.rr[0];qq=self.q.get(s)
            if not qq:self.rr.popleft();self.deficit[s]=0;continue
            if self.deficit[s]<qq[0][2]:
                self.deficit[s]+=self.quantum;self.rr.rotate(-1);continue
            pk=qq.popleft();self.deficit[s]-=pk[2]
            if not qq:self.rr.remove(s);self.deficit[s]=0
            return pk
        return None
    def _start_next(self):
        if self.busy:return
        while True:
            pk=self._pop()
            if pk is None:return
            mid,i,w=pk;self.waiting_bytes-=w;r=self.records[mid]
            if r.status!='queued':continue                     # expired/replaced/dropped: skipped, no link time
            r.started=True
            d=math.ceil(8*w*1_000_000/self.capacity_bps);self.busy=True
            self._sched(self.now_us+d,'tx_done',(mid,i,w));return
    # ---------- API (mirrors frozen SharedLink) ----------
    def offer(self,*,stream,payload_bytes,stamp_us,reliability='best_effort',expiry_us='stream'):
        self.next_id+=1;mid=self.next_id;units=self._packets(mid,payload_bytes)
        exp=self.expiry.get(stream) if expiry_us=='stream' else expiry_us
        r=Rec(mid,stream,payload_bytes,stamp_us,len(units),None);self.records[mid]=r;self.bytes_by[(stream,'offered')]+=payload_bytes
        if self.disc=='keeplast':          # replace this stream's older messages that have not started
            for m2,r2 in list(self.records.items()):
                if m2!=mid and r2.stream==stream and r2.status=='queued' and not r2.started:self._finish(r2,'replaced')
        need=sum(u[2] for u in units)
        if need+self.waiting_bytes>self.qlim:self._finish(r,'queue_drop');return mid
        for u in units:self._push(stream,u)
        self.waiting_bytes+=need
        if exp is not None:
            r.deadline=self.now_us+exp;self._sched(r.deadline,'expire',mid)
        self._start_next();return mid
    def advance(self,target):
        if target<self.now_us:raise ValueError('backwards')
        while self.events and self.events[0][0]<=target:
            t,_,ev,data=heapq.heappop(self.events);self.now_us=t
            if ev=='tx_done':
                mid,i,w=data;self.busy=False;self.wire['bytes_total']+=w;r=self.records[mid]
                if r.status=='queued':self._sched(t+self.delay,'arrival',mid)
                self._start_next()
            elif ev=='arrival':
                r=self.records[data]
                if r.status=='queued':
                    r.got+=1
                    if r.got==r.total:self._finish(r,'delivered')
            elif ev=='expire':
                r=self.records[data]
                if r.status=='queued':self._finish(r,'expired')
        self.now_us=target
