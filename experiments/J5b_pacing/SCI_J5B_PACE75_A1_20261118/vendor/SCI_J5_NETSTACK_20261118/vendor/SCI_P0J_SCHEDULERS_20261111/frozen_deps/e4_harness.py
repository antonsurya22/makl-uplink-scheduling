#!/usr/bin/env python3
"""E4 harness: deterministic replay of the shared uplink with a synthetic simulation-time clock.

Offered traces come from E3-B C0 runs, where nothing was dropped, so every logged edge frame and scan IS an offer:
  * camera: every received frame (capture stamp; payload = image bytes + 64 B CDR overhead, measured exactly in E3);
    frames before the goal are STOP-Q2 warm-up frames (best effort), after it the run's profile;
  * scan: 5 Hz stamp grid (phase from the logged stamps), 2,944 B serialized (2 MTU packets, 3,000 B on the wire);
  * probe ACK: 96 B, one per probe; probes every 0.5 s from bridge start (first 5 s report - 5 s); the ACK is offered
    one /clock tick after its probe (downlink 35 +/- 5 ms < one tick).
Timing model (from the bridge code and the E3-B logs): /clock ticks every 0.1 s; capacity boundaries are applied at
their exact sim time; a message is offered at the first tick >= its stamp; a delivery is observed (published and
received at the edge) at the first tick >= its completion time. Order inside a tick follows the bridge:
advance -> scans -> flush -> camera -> flush -> probe ACK.
"""
from __future__ import annotations
import json,math,re,statistics as st,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'frozen'));sys.path.insert(0,str(HERE))
from shared_link import SharedLink                     # frozen GATE_B1 code, SHA256 04f29070...
from link_disciplines import DisciplineLink
TICK=100_000;SCAN_PAYLOAD=2944;ACK_PAYLOAD=96;CDR=64;QLIM=768_000;DELAY=20_000
def us(x):return int(round(x*1e6))
def jl(p):return [json.loads(l) for l in open(p) if l.strip()]
def load_trace(tdir,res):
    tdir=Path(tdir);fr=jl(tdir/'E3B_FRAMES.jsonl');sc=jl(tdir/'E3B_EDGE_SCAN.jsonl')
    log=(tdir/'c52_joint_bridge.log').read_text(errors='replace')
    first=int(re.search(r'C2_BRIDGE sim_us=(\d+)',log).group(1));start=first-5_000_000
    st_=sorted(r['stamp_s'] for r in sc);ph=us(st_[0])%200_000
    scans=list(range(us(st_[0])-200_000*int((us(st_[0])-start)//200_000),us(st_[-1])+1,200_000))
    frames=sorted((us(r['stamp_s']),r['bytes']+CDR) for r in fr)
    return {'trial_id':res['trial_id'],'arm':res['arm'],'start_us':start,'goal_us':us(res['goal_sim_s']),'trigger_us':us(res['trigger_sim_s']),
            'end_us':us(res['terminal_sim_s']),'frames':frames,'scans':scans,'profile':'Q1' if res['arm']=='F-N1' else 'Q2'}
def replay(tr,*,capacity_mbps,duration_s,link='frozen',discipline='fifo',cam_expiry=None,scan_expiry=None,
           cam_mode='as_logged',horizon_after_s=5.0):
    """cam_mode 'as_logged': Q1 frames reliable (1 s expiry) after the goal, warm-up frames best effort (= frozen bridge).
    For the factorial, cam_expiry/scan_expiry (seconds or None) override reliability on the DisciplineLink."""
    T=tr['trigger_us'];G=tr['goal_us'];t0=(G//TICK)*TICK;t1=T+us(duration_s+horizon_after_s)
    if link=='frozen':
        L=SharedLink(name='uplink',seed=0,capacity_bps=3_000_000,queue_limit_bytes=QLIM,delay_us=DELAY)
    else:
        L=DisciplineLink(capacity_bps=3_000_000,queue_limit_bytes=QLIM,delay_us=DELAY,discipline=discipline)
    L.advance(t0)
    bounds=[(T,int(capacity_mbps*1e6)),(T+us(duration_s),3_000_000)] if capacity_mbps is not None else []
    fi=0;si=0;frames=[f for f in tr['frames'] if f[0]>=t0-TICK];scans=[s for s in tr['scans'] if s>=t0-TICK]
    probes=[];nextp=tr['start_us']+((t0-tr['start_us'])//500_000+1)*500_000
    meta={};recv=[];queue=[];seen=set()
    def offer(stream,payload,stamp,now):
        if link=='frozen':
            rel='best_effort'
            if stream=='camera' and stamp>=G and tr['profile']=='Q1':rel='reliable'
            mid=L.offer(stream=stream,payload_bytes=payload,stamp_us=now,reliability=rel)
        else:
            exp=None
            if stream=='camera':exp=None if cam_expiry is None else us(cam_expiry)
            if stream=='scan':exp=None if scan_expiry is None else us(scan_expiry)
            if cam_mode=='as_logged' and stream=='camera':exp=1_000_000 if (stamp>=G and tr['profile']=='Q1') else None
            mid=L.offer(stream=stream,payload_bytes=payload,stamp_us=now,expiry_us=exp)
        meta[mid]=(stream,stamp,payload,now)
    def flush(now):
        for mid,(stream,stamp,payload,offered) in meta.items():
            if mid in seen:continue
            r=L.records[mid];stt=r.status;fin=r.finished_us if link=='frozen' else r.finished
            if stt=='delivered' and fin<=now:recv.append((stream,now,stamp,offered,payload));seen.add(mid)
            elif stt in ('queue_drop','expired','channel_loss','replaced'):seen.add(mid)
    t=t0
    while t<=t1:
        while bounds and bounds[0][0]<=t:
            b,c=bounds.pop(0)
            if L.now_us<=b:L.advance(b)
            L.capacity_bps=c
        L.advance(t)
        while si<len(scans) and scans[si]<=t:offer('scan',SCAN_PAYLOAD,scans[si],t);si+=1
        L.advance(t);flush(t)
        while fi<len(frames) and frames[fi][0]<=t:offer('camera',frames[fi][1],frames[fi][0],t);fi+=1
        flush(t)
        for p in [p for p in probes if p+TICK<=t]:offer('c52_probe_ack',ACK_PAYLOAD,p,t);probes.remove(p)
        if t>=nextp:probes.append(t);nextp+=500_000
        queue.append((t,L.waiting_bytes))
        t+=TICK
    status={}
    for mid,(stream,stamp,payload,offered) in meta.items():
        r=L.records[mid];status[mid]=(stream,stamp,payload,offered,r.status,(r.finished_us if link=='frozen' else r.finished))
    return {'recv':recv,'queue':queue,'status':status,'T':T,'dur_us':us(duration_s),'G':G,'wire':L.wire['bytes_total']}
# ---------------- metrics ----------------
def age_grid(recv,stream,a,b,dt=TICK):
    """10 Hz grid of t - stamp of the newest message of `stream` received by t (inf before the first)."""
    r=sorted((tt,s) for st_,tt,s,_,_ in recv if st_==stream);out=[];i=0;newest=None;t=a
    while t<b:
        while i<len(r) and r[i][0]<=t:newest=r[i][1] if newest is None else max(newest,r[i][1]);i+=1
        out.append(math.inf if newest is None else (t-newest)/1e6);t+=dt
    return out
def pct(xs,q):
    xs=sorted(xs);return xs[min(len(xs)-1,int(math.ceil(q*len(xs)))-1)] if xs else None
def metrics(R,window='event+2'):
    T=R['T'];a=T;b=T+R['dur_us']+(2_000_000 if window=='event+2' else 0)
    sg=age_grid(R['recv'],'scan',a,b);cg=age_grid(R['recv'],'camera',a,b);n=len(sg)
    fin=[x for x in sg if math.isfinite(x)]
    camr=[(tt,s) for st_,tt,s,_,_ in R['recv'] if st_=='camera' and a<=tt<b]
    sec=(b-a)/1e6
    off=lambda s:[v for v in R['status'].values() if v[0]==s and a<=v[3]<b]
    so=off('scan');co=off('camera')
    cnt=lambda L,stt:sum(1 for v in L if v[4]==stt)
    byt=lambda s,stt:sum(v[2] for v in R['status'].values() if v[0]==s and v[4]==stt and a<=v[3]<b)
    first=lambda g,th:next(((k*TICK)/1e6 for k,x in enumerate(g) if x>th),None)
    q=[w for tt,w in R['queue'] if a<=tt<b]
    sd=[(tt-s)/1e6 for st_,tt,s,_,_ in R['recv'] if st_=='scan' and a<=s<b]
    return {'scan_age_med_s':pct(fin,.5),'scan_age_p95_s':pct(sg,.95) if fin else math.inf,'scan_age_max_s':max(sg),
            'scan_share_gt_0p5':sum(1 for x in sg if x>0.5)/n,'scan_offered':len(so),'scan_delivered':cnt(so,'delivered'),
            'scan_expired':cnt(so,'expired'),'scan_dropped':cnt(so,'queue_drop')+cnt(so,'replaced'),
            'scan_delay_med_s':pct(sd,.5),'scan_delay_max_s':max(sd) if sd else None,
            'cam_V':sum(1 for x in cg if x>1.0)/n,'cam_F_fps':sum(1 for tt,s in camr if (tt-s)<=1_000_000)/sec,'cam_raw_fps':len(camr)/sec,
            'cam_offered':len(co),'cam_delivered':cnt(co,'delivered'),'cam_delivered_in_window':len(camr),
            'scan_delivered_in_window':sum(1 for st_,tt,s,_,_ in R['recv'] if st_=='scan' and a<=tt<b),'cam_expired':cnt(co,'expired'),'cam_dropped':cnt(co,'queue_drop')+cnt(co,'replaced'),
            'max_backlog_bytes':max(q) if q else 0,'max_backlog_2hz_bytes':max([w for tt,w in R['queue'] if a<=tt<b and (tt-a)%500_000==0] or [0]),
            'bytes_expired_scan':byt('scan','expired'),'bytes_expired_cam':byt('camera','expired'),
            'bytes_dropped_scan':byt('scan','queue_drop')+byt('scan','replaced'),'bytes_dropped_cam':byt('camera','queue_drop')+byt('camera','replaced'),
            't_first_scan_age_gt_0p5_s':first(sg,0.5),'t_first_cam_aoi_gt_1_s':first(cg,1.0)}
