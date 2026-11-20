#!/usr/bin/env python3
"""E2 pure decision logic (no ROS, no torch). Reproduces the R2 contracts exactly:

* 9 actions: NORMAL/SLOW/STOP x Q0/Q1/Q2 (id = 3*mode + profile)       [D4 action_logic]
* motion caps: NORMAL 0.15 m/s, SLOW 0.08 m/s, STOP 0; |w| <= 0.5 rad/s [D4 validate_attestation]
* decision mask: STOP-only when raw scan <= max(0.70, 0.39+1.75v+v^2/0.4+0.06),
  stale (>0.5 s) or missing scan, or any contact                       [V3 adapter_safe_goal._mask]
* 10-D observation and normalisation                                    [D4 measured_state]
* camera deadline = camera AoI > 0.60 s (reward v3 camera_target_s)     [service_reward_v3]
Arms: F-N1=1, F-N2=2, F-S1=4, F-S2=5 (fixed); RULE (sensor rule); JOINT (frozen DDQN, greedy).
"""
from __future__ import annotations
import hashlib,json,math

ACTION_NAMES=('NORMAL-Q0','NORMAL-Q1','NORMAL-Q2','SLOW-Q0','SLOW-Q1','SLOW-Q2','STOP-Q0','STOP-Q1','STOP-Q2')
LIN_CAP=(0.15,0.08,0.0)
ANG_CAP=0.5
FIXED={'F-N1':1,'F-N2':2,'F-S1':4,'F-S2':5}
ARMS=('F-N1','F-N2','F-S1','F-S2','RULE','JOINT')
CAMERA_DEADLINE_S=0.60
SCAN_DEADLINE_S=0.35
NORMALIZERS=(.70,.10,1.,18.43712,3.,10.,3.5)
FROZEN_CKPT={17:'91124ebea033dc539397b5f91857f88d72bc7c85b77e063a4318c1920e93906c',
             31:'8cd1f2a93a7b45692e9836584349ff2dee39d764eb17ef1c2fad1769cf98f41c',
             47:'c329ccd48ac4e8b3b4d2b7aa54437f9a15c1d0c4577da4e72b7d6ace9d0a142b'}
CKPT_REL={s:f'results/SCI_V3D_R2_PROGRESSIVE_TRAIN_20261028/seed{s}/checkpoints/ep030.pt' for s in FROZEN_CKPT}
C1_PLAN={'normal_mbps':3.0,'congest_mbps':0.25,'recovery_mbps':3.0,
         'congest_after_trigger_s':0.0,'recover_after_trigger_s':8.0}

def mode_of(a):
    if type(a) is not int or not 0<=a<=8:raise ValueError('INVALID_ACTION')
    return a//3
def profile_of(a):mode_of(a);return a%3

def braking_threshold(speed_mps):
    v=max(0.0,float(speed_mps))
    return max(0.70,0.39+v*1.75+v*v/(2*0.20)+0.06)

def decision_mask(raw_scan_m,scan_age_s,speed_mps,contacts):
    stop_only=[False]*6+[True]*3
    if contacts or raw_scan_m is None or scan_age_s is None or not math.isfinite(float(scan_age_s)) or scan_age_s>0.5:
        return stop_only
    if not math.isfinite(float(raw_scan_m)) or float(raw_scan_m)<=braking_threshold(speed_mps):
        return stop_only
    return [True]*9

def choose_fixed(arm,mask):
    a=FIXED[arm]
    if mask[a]:return a,False
    stop=6+a%3            # keep the arm's profile while the mask forces STOP
    if mask[stop]:return stop,True
    raise RuntimeError('NO_SAFE_STOP_AVAILABLE')

from e2_event_logic import AdaptiveRule   # pre-registered rule, used unchanged

def rule_act(rule,distance_m,scan_age_s,mask):
    """Wrap the unchanged AdaptiveRule; invalid sensor quantities -> STOP-Q2 (fail-closed)."""
    try:return rule.act(float(distance_m),float(scan_age_s),[bool(x) for x in mask])
    except (ValueError,TypeError):
        if mask[8]:return 8,True
        raise RuntimeError('NO_SAFE_STOP_AVAILABLE')

def build_observation(network,scan_age_s,camera_age_s,dist_obstacle_m,dist_goal_m,initial_goal_m,executed_action):
    """Exact D4 measured_state.build_observation semantics. network: dict latency,jitter,packet_loss,bandwidth."""
    if type(executed_action) is not int or not 0<=executed_action<=8:raise ValueError('UNATTESTED_ACTION_INVALID')
    vals=[float(network[k]) for k in ('latency','jitter','packet_loss','bandwidth')]
    vals+=[float(scan_age_s),float('nan') if camera_age_s is None else float(camera_age_s),float(dist_obstacle_m)]
    init=float(initial_goal_m);dist=float(dist_goal_m)
    if not math.isfinite(init) or init<=1e-3 or not math.isfinite(dist) or dist<0:raise ValueError('NAV2_SCOPED_GOAL_DISTANCE_INVALID')
    vals.append(dist)
    for i,x in enumerate(vals):
        if math.isnan(x) and i==5:continue
        if not math.isfinite(x) or x<0:raise ValueError('NONFINITE_OR_NEGATIVE_REAL_SIGNAL_'+str(i))
    if vals[2]>1+1e-6:raise ValueError('PACKET_LOSS_NOT_RATIO')
    c=lambda x:max(0.,min(1.,float(x)))
    raw=vals+[float(executed_action//3),float(executed_action%3)]
    norm=[c(vals[0]/.70),c(vals[1]/.10),c(vals[2]),c(vals[3]/18.43712),c(vals[4]/3.),
          1.0 if math.isnan(vals[5]) else c(vals[5]/10.),c(vals[6]/3.5),c(dist/init),
          float(executed_action//3)/2.,float(executed_action%3)/2.]
    return raw,norm

def schedule_sha(condition):
    if condition=='C0':obj={'condition':'C0','capacity_mbps':3.0,'trigger':'none'}
    elif condition=='C1':obj={'condition':'C1','plan':C1_PLAN,'trigger':'actor crossing_trigger_ack (robot x>=1.30)'}
    else:raise ValueError('BAD_CONDITION')
    return hashlib.sha256(json.dumps(obj,sort_keys=True).encode()).hexdigest()

def percentile(xs,q):
    xs=sorted(x for x in xs if x is not None and math.isfinite(x))
    if not xs:return None
    k=(len(xs)-1)*q;f=math.floor(k);c=min(f+1,len(xs)-1)
    return xs[f]+(xs[c]-xs[f])*(k-f)

def rising_edges(flags):
    n=0;prev=False
    for f in flags:
        if f and not prev:n+=1
        prev=bool(f)
    return n

def run_order(matrix_rows):
    """Deterministic execution order: per block, C0 fixed/RULE by 'order', then C1, then JOINT repeat==block."""
    out=[]
    for b in ('1','2','3','4','5'):
        for cond in ('C0','C1'):
            rows=[r for r in matrix_rows if r['block']==b and r['condition']==cond and r['arm']!='JOINT']
            out+=sorted(rows,key=lambda r:int(r['order']))
        for cond in ('C0','C1'):
            out+=sorted([r for r in matrix_rows if r['arm']=='JOINT' and r['block']==b and r['condition']==cond],key=lambda r:int(r['seed']))
    if len(out)!=len(matrix_rows) or len({r['trial_id'] for r in out})!=len(matrix_rows):raise RuntimeError('RUN_ORDER_INCOMPLETE')
    return out
