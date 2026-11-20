#!/usr/bin/env python3
"""E3-B (does the network reach motion?): pure logic, no ROS. Frozen 31 Oct 2026 BEFORE any E3-B run.

Nav2 costmaps read /gate_b2/scan_edge (Gate B design); the safety layer stays on raw /scan (byte-pinned).
Levels (from E2 C0 demand, Part 0): D1 = 0.2663, D2 = 0.1563 Mbps.
  LA = 0.21, LB = 0.18 (as E3), LC = round(0.85*D2, 2) = 0.13 Mbps (neither profile fits).
Congestion: event-triggered at the pedestrian trigger (robot x >= 1.30 m) and held for 25 s (E3 data: in an 8 s
window the robot is stopped for the pedestrian in 7-8 of 10 decisions and the last safety STOP is at <= +20.2 s, so
an 8 s event could never reach motion). Window W = trigger -> +25 s.
Rule thresholds: theta_up = ceil_0.05(max(0.20, d_C0_max + 0.05)) with d_C0_max = 0.2397 s (largest RULE-NET d in any
E3 C0 run, whole episode: E3_R1_C0_F-N1) -> 0.30 s; theta_down = 0.10 s.
"""
from __future__ import annotations
import hashlib,json,math
import e3_policy_logic as L3

D1_MBPS=L3.D1_MBPS;D2_MBPS=L3.D2_MBPS
LEVELS={'C0':None,'LA':0.21,'LB':0.18,'LC':0.13}
assert LEVELS['LC']==round(0.85*D2_MBPS,2) and LEVELS['LA']==L3.LEVELS['LA'] and LEVELS['LB']==L3.LEVELS['LB']
D_C0_MAX_E3=0.23967480522187778
def _ceil05(x):return math.ceil(round(x/0.05,9))*0.05
THETA_UP=round(_ceil05(max(0.20,D_C0_MAX_E3+0.05)),2);THETA_DOWN=0.10
assert THETA_UP==0.30
CONGESTION_S=25.0;WINDOW_S=25.0;GRID_DT=0.1;NAV_STALE_S=0.5;SLO_S=1.0
ARMS=('F-N1','F-N2','F-S2','RULE-COMM','RULE-NAV','RULE-JOINT')
FIXED={'F-N1':1,'F-N2':2,'F-S2':5}
BASE=1                                   # NORMAL-Q1
RESPONSE={'RULE-COMM':(2,),'RULE-NAV':(4,),'RULE-JOINT':(2,5)}   # NORMAL-Q2 | SLOW-Q1 | NORMAL-Q2 -> SLOW-Q2
NAMES=('NORMAL-Q0','NORMAL-Q1','NORMAL-Q2','SLOW-Q0','SLOW-Q1','SLOW-Q2','STOP-Q0','STOP-Q1','STOP-Q2')

def plan(level):
    c=LEVELS[level]
    return {'normal_mbps':3.0,'congest_mbps':0.25 if c is None else c,'recovery_mbps':3.0,
            'congest_after_trigger_s':0.0,'recover_after_trigger_s':CONGESTION_S}
def schedule_sha(level):
    obj={'level':level,'capacity_mbps':LEVELS[level],'trigger':'none' if LEVELS[level] is None else
         'actor crossing_trigger_ack (robot x>=1.30)','duration_s':CONGESTION_S}
    return hashlib.sha256(json.dumps(obj,sort_keys=True).encode()).hexdigest()
with_mask=L3.with_mask            # requested action, or STOP with the SAME profile when masked
delay_signal=L3.delay_signal      # d = sender backlog*8 / delivered wire rate (2 s), robot-side only

class EscalationRule:
    """Shared trigger/hold/backoff/probe; arms differ ONLY in the response state(s).
    Inputs: sim time and d. Never reads phase label, trigger, schedule, ground truth or a_nav."""
    B0=4.0;BMAX=16.0;PROBE_N=3;GOOD_N=3
    def __init__(self,arm,theta_up=THETA_UP,theta_down=THETA_DOWN):
        if arm not in RESPONSE:raise ValueError('UNKNOWN_RULE_'+str(arm))
        self.arm=arm;self.resp=RESPONSE[arm];self.up=theta_up;self.down=theta_down
        self.level=0                      # 0 = base NORMAL-Q1; k>=1 = response state resp[k-1]
        self.b=self.B0;self.hold_until=None;self.since_probe=None;self.good=0
        self.resp_d=[]                    # d values at/after entering the CURRENT first response state
        self.escalations=0;self.log=[]
    @property
    def action(self):return BASE if self.level==0 else self.resp[self.level-1]
    def _respond(self,sim_s,failed_probe):
        if failed_probe:self.b=min(2*self.b,self.BMAX)
        self.level=1;self.hold_until=sim_s+self.b;self.since_probe=None;self.good=0
        return 'failed_probe_backoff_x2' if failed_probe else 'respond'
    def step(self,sim_s,d):
        ev=None
        if d is None:ev='signal_unavailable_keep'
        elif self.level==0:
            if d>self.up:
                ev=self._respond(sim_s,self.since_probe is not None and self.since_probe<self.PROBE_N)
                self.resp_d=[d]
            else:
                self.good+=1
                if self.since_probe is not None:
                    self.since_probe+=1
                    if self.since_probe>=self.PROBE_N:self.since_probe=None
                if self.good>=self.GOOD_N and self.b!=self.B0:self.b=self.B0;ev='backoff_reset'
        else:
            if self.level==1:self.resp_d.append(d)
            if self.hold_until is not None and sim_s>=self.hold_until and d<self.down:
                self.level=0;self.since_probe=0;self.good=0;self.resp_d=[];ev='probe_q1_normal'
            elif (self.level==1 and len(self.resp)>1 and d>self.up and len(self.resp_d)>=3
                  and self.resp_d[-1]>self.resp_d[-2]>self.resp_d[-3]):
                self.level=2;self.hold_until=sim_s+self.b;self.escalations+=1;ev='escalate'
        self.log.append({'sim_s':sim_s,'d':d,'action':self.action,'name':NAMES[self.action],'b':self.b,'event':ev})
        return self.action

def requested_action(arm,rule,sim_s,d):
    if arm in FIXED:return FIXED[arm]
    return rule.step(sim_s,d)

# ---------------- per-run metrics (10 Hz sim grid) ----------------
def hold_grid(rows,key,t0,t1,dt=GRID_DT,max_gap=0.35):
    """Zero-order hold of rows[{'sim_s':..,key:..}] onto [t0,t1); None if the newest sample is older than max_gap."""
    rows=sorted((r for r in rows if r.get(key) is not None and r.get('sim_s') is not None),key=lambda r:r['sim_s'])
    out=[];i=0;last=None;n=int(round((t1-t0)/dt))
    for k in range(n):
        t=t0+k*dt
        while i<len(rows) and rows[i]['sim_s']<=t+1e-9:last=rows[i];i+=1
        out.append(None if last is None or t-last['sim_s']>max_gap else last[key])
    return out
def a_nav_grid(edge,t0,t1,dt=GRID_DT):
    """edge: [(receipt_sim_s, stamp_s)] of /gate_b2/scan_edge. a_nav(t) = t - stamp of the newest scan received by t."""
    g=L3.aoi_grid(edge,t0,t1-dt+1e-9,dt);return [a for _,a in g][:int(round((t1-t0)/dt))]
def run_metrics(grid,edge,frames,steps,t0,t_goal_success,window=WINDOW_S):
    """All metrics over W=[t0,t0+window). grid: E3B_GRID rows. Returns a flat dict (None where not computable)."""
    t1=t0+window;n=int(round(window/GRID_DT))
    an=a_nav_grid(edge,t0,t1);v=hold_grid(grid,'v_mps',t0,t1);pol=hold_grid(grid,'cmd_policy_v',t0,t1)
    fin=hold_grid(grid,'cmd_final_v',t0,t1);cl=hold_grid(grid,'clearance_m',t0,t1);raw=hold_grid(grid,'raw_scan_min_m',t0,t1)
    stale=[a is not None and a>NAV_STALE_S for a in an]
    p3=sum(abs(x)*GRID_DT for x,s in zip(v,stale) if s and x is not None)
    stop=[p is not None and f is not None and p>0.01 and f<=0.005 for p,f in zip(pol,fin)]
    hold=[p is not None and abs(p)<0.01 for p in pol]
    episodes=0;prev=False
    for s in stop:
        if s and not prev:episodes+=1
        prev=s
    guard070=0;prev=False
    for r in raw:
        f=r is not None and r<0.70
        if f and not prev:guard070+=1
        prev=f
    fin_an=sorted(a for a in an if a is not None and math.isfinite(a))
    def pct(xs,q):return None if not xs else xs[min(len(xs)-1,int(math.ceil(q*len(xs)))-1)]
    useful,rawfps=L3.F_metric([(r,s) for r,s in frames],t0,t1)
    return {'P1_s':None if t_goal_success is None else t_goal_success-t0,
            'P2_stop_s':round(GRID_DT*sum(stop),3),'P2_stop_episodes':episodes,
            'P2_masked_decisions':sum(1 for r in steps if r.get('masked_fallback') and t0<=r['sim_s']<t1),
            'guard_070_episodes':guard070,'nav2_hold_s':round(GRID_DT*sum(hold),3),
            'P3_m':round(p3,4),'S':sum(stale)/n,'F_fps':useful,'raw_fps':rawfps,
            'V':L3.V_metric([(r,s) for r,s in frames],t0,t1) if frames else 1.0,
            'a_nav_med_s':pct(fin_an,.5),'a_nav_p95_s':pct(fin_an,.95),'a_nav_max_s':fin_an[-1] if fin_an else None,
            'min_clearance_W_m':min((c for c in cl if c is not None),default=None),
            'min_raw_W_m':min((r for r in raw if r is not None),default=None),
            'grid_coverage':sum(1 for x in v if x is not None)/n,
            'slow_s':round(GRID_DT*sum(1 for r in hold_grid(grid,'mode',t0,t1) if r==1),3)}

def clear_difference(xs,ys,J):
    """+1 if xs clearly larger than ys, -1 if clearly smaller, 0 otherwise. Both: per-rep values do not overlap AND
    |mean difference| > J. Lists with None (failed/unmeasured runs) -> 0 (cannot be judged)."""
    if not xs or not ys or any(x is None for x in xs) or any(y is None for y in ys):return 0
    mx,my=sum(xs)/len(xs),sum(ys)/len(ys)
    if min(xs)>max(ys) and mx-my>J:return 1
    if max(xs)<min(ys) and my-mx>J:return -1
    return 0
JITTER_FLOOR={'P1_s':1.0,'P2_stop_s':1.0,'P3_m':0.05,'F_fps':0.3}
