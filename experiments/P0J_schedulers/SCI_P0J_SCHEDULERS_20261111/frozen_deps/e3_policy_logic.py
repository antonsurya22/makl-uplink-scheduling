#!/usr/bin/env python3
"""E3 operating-envelope test: pure logic (no ROS). Frozen 31 Oct 2026 BEFORE any E3 run.

D1/D2 = mean total uplink wire rate (camera + scan + probe ACK) measured in E2 C0 F-N1 / F-N2
(NetworkState.bandwidth, 0.5 s windows at every decision; n = 194 each):
  D1 = 0.2663 Mbps (p95 0.3209), D2 = 0.1563 Mbps (p95 0.1953)
c_A = round((D1+D2)/2, 2) = 0.21 Mbps ; c_B = round(1.15*D2, 2) = 0.18 Mbps (run because 0.18 <= 0.21-0.02)
All levels >= 0.15 Mbps, so the event-triggered plan validator (0.15..0.50 Mbps) is NOT changed.
"""
from __future__ import annotations
import hashlib,json,math

D1_MBPS=0.2663;D2_MBPS=0.1563
LEVELS={'C0':None,'L0.25':0.25,'LA':0.21,'LB':0.18}
assert LEVELS['LA']==round((D1_MBPS+D2_MBPS)/2,2) and LEVELS['LB']==round(1.15*D2_MBPS,2)
assert LEVELS['LB']<=LEVELS['LA']-0.02+1e-12
SLO_S=1.0;SECONDARY_DEADLINE_S=0.60;WINDOW_S=10.0;CONGESTION_S=8.0
ARMS=('F-N1','F-N2','ORACLE','RULE-NET')
FIXED={'F-N1':1,'F-N2':2}

def plan(level):
    """Event-triggered plan for the C52 bridge. C0 gets a valid plan that is never triggered."""
    c=LEVELS[level]
    return {'normal_mbps':3.0,'congest_mbps':0.25 if c is None else c,'recovery_mbps':3.0,
            'congest_after_trigger_s':0.0,'recover_after_trigger_s':8.0}
def schedule_sha(level):
    obj={'level':level,'capacity_mbps':LEVELS[level],'trigger':'none' if LEVELS[level] is None else
         'actor crossing_trigger_ack (robot x>=1.30)','duration_s':CONGESTION_S}
    return hashlib.sha256(json.dumps(obj,sort_keys=True).encode()).hexdigest()

def with_mask(action,mask):
    """Executed action: requested NORMAL action, or STOP with the SAME profile when the mask forbids motion."""
    if mask[action]:return action,False
    stop=6+action%3
    if mask[stop]:return stop,True
    raise RuntimeError('NO_SAFE_STOP_AVAILABLE')

def oracle_profile(sim_s,trigger_s,congested_level):
    """Upper bound only: knows the schedule. Q2 inside [trigger, trigger+8) at congested levels, else Q1."""
    if congested_level and trigger_s is not None and trigger_s<=sim_s<trigger_s+CONGESTION_S:return 2
    return 1

def delay_signal(backlog_bytes,wire_samples,now_sim_s,window_s=2.0):
    """Signal (ii): d = backlog*8 / delivered rate over the last 2 s. wire_samples: [(sim_s, cumulative wire bytes)].
    Returns (d_seconds or None, delivered_bps or None). Uses only robot-side quantities (sender queue, sent bytes)."""
    if backlog_bytes is None:return None,None
    pts=[(t,b) for t,b in wire_samples if now_sim_s-window_s-0.6<=t<=now_sim_s+1e-9]
    if len(pts)<2:return None,None
    (t0,b0),(t1,b1)=pts[0],pts[-1]
    if t1-t0<1.0:return None,None
    rate=max(0.,(b1-b0)*8/(t1-t0))
    if backlog_bytes<=0:return 0.0,rate
    if rate<=0:return math.inf,rate
    return backlog_bytes*8/rate,rate

class RuleNet:
    """Frozen RULE-NET (brief 3.3). Q1 start, backoff b=4 s; Q1: d>0.30 -> Q2 for b s; Q2: after hold and d<0.10 ->
    probe Q1; d>0.30 within 3 decisions after a probe -> Q2 and b=min(2b,16); 3 Q1 decisions without d>0.30 -> b=4.
    Never uses phase label, trigger, schedule or ground truth. d=None (signal unavailable) -> keep profile."""
    HI=0.30;LO=0.10;B0=4.0;BMAX=16.0;PROBE_N=3;GOOD_N=3
    def __init__(self):
        self.profile=1;self.b=self.B0;self.hold_until=None;self.since_probe=None;self.good=0;self.log=[]
    def step(self,sim_s,d):
        ev=None
        if d is None:ev='signal_unavailable_keep'
        elif self.profile==1:
            if d>self.HI:
                if self.since_probe is not None and self.since_probe<self.PROBE_N:
                    self.b=min(2*self.b,self.BMAX);ev='failed_probe_backoff_x2'
                else:ev='congestion_detected'
                self.profile=2;self.hold_until=sim_s+self.b;self.since_probe=None;self.good=0
            else:
                self.good+=1
                if self.since_probe is not None:
                    self.since_probe+=1
                    if self.since_probe>=self.PROBE_N:self.since_probe=None
                if self.good>=self.GOOD_N and self.b!=self.B0:self.b=self.B0;ev='backoff_reset'
        else:
            if self.hold_until is not None and sim_s>=self.hold_until and d<self.LO:
                self.profile=1;self.since_probe=0;self.good=0;ev='probe_q1'
        self.log.append({'sim_s':sim_s,'d':d,'profile':self.profile,'b':self.b,'event':ev})
        return self.profile

# ---------------- service metrics (edge-side, sim time) ----------------
def aoi_grid(frames,t_start,t_end,dt=0.1):
    """frames: [(receipt_sim_s, capture_stamp_s)]. AoI(t) = t - capture stamp of the newest frame received by t.
    Before the first receipt AoI is +inf (counts as violation)."""
    fr=sorted(frames);out=[];i=0;newest=None;n=int(round((t_end-t_start)/dt))
    for k in range(n+1):
        t=t_start+k*dt
        while i<len(fr) and fr[i][0]<=t+1e-9:
            newest=fr[i][1] if newest is None else max(newest,fr[i][1]);i+=1
        out.append((t,math.inf if newest is None else max(0.,t-newest)))
    return out
def V_metric(frames,t_start,t_end,slo=SLO_S):
    g=aoi_grid(frames,t_start,t_end);return sum(1 for _,a in g if a>slo)/len(g)
def F_metric(frames,t_start,t_end,slo=SLO_S):
    dur=t_end-t_start
    useful=sum(1 for r,s in frames if t_start<=r<t_end and r-s<=slo)
    raw=sum(1 for r,s in frames if t_start<=r<t_end)
    return useful/dur,raw/dur

def dominates(p,x):
    """Brief 3.6: P dominates X if V(P)<=V(X)+2pp and F(P)>=0.95F(X), and (V(P)<=V(X)-5pp or F(P)>=1.20F(X))."""
    keep=p['V']<=x['V']+0.02 and p['F']>=0.95*x['F']
    gain=p['V']<=x['V']-0.05 or p['F']>=1.20*x['F']
    return keep and gain

def run_order(matrix_rows):
    out=[]
    for rep in ('1','2','3'):
        for lev in ('C0','L0.25','LA','LB'):
            rows=sorted([r for r in matrix_rows if r['rep']==rep and r['level']==lev],key=lambda r:int(r['order']))
            out+=rows
    if len(out)!=len(matrix_rows) or len({r['trial_id'] for r in out})!=len(out):raise RuntimeError('E3_RUN_ORDER_INCOMPLETE')
    return out
