#!/usr/bin/env python3
"""E5 per-run measures (pre-specified; used by the batch for pilot acceptance and halts, and by e5_analysis.py).
Uses the frozen E3-B helpers (e3b_policy_logic.hold_grid / a_nav_grid / GRID_DT, e2_policy_logic.braking_threshold).
Definitions:
  centre distance c(t) = E3B_GRID clearance_m + 0.22;  raw-equivalent distance r(t) = c(t) - 0.10 (pedestrian radius)
  ENCOUNTER t_enc = first 10 Hz grid point after the pedestrian starts with r(t) <= braking_threshold(v(t))
                    (the decision-mask threshold of the safety layer, max(0.70, 0.39+1.75v+v^2/0.4+0.06))
  Eq. 1 violation   = moving grid point (|v| > 0.01) with |v| > v_max(a, d), d = clearance_m; u primary = nominal speed
                      (constant), secondary = actual actor speed from consecutive positions; b = 0.2 / 0.5 / 1.0;
                      a = a_nav (planner) or the raw local scan age (control); dt = 0.1 s
  CONTACT class     = robot-moving if robot |v| > 0.02 m/s at any grid point in the 0.5 s before the first actor contact,
                      else pedestrian-initiated
  W = network trigger -> +25 s; EP = goal -> end of run"""
import json,math,sys
from pathlib import Path
E3B=Path.home()/'ros2_ws/SCI_E3B_EDGE_NAV_RUNNER_20261031'
if str(E3B) not in sys.path:sys.path.insert(0,str(E3B))
import e3b_policy_logic as L
from e2_policy_logic import braking_threshold
DT=0.1;BS=(0.2,0.5,1.0);VMOVE=0.01
def jl(p):
    p=Path(p);return [json.loads(l) for l in p.read_text(errors='replace').splitlines() if l.strip()] if p.is_file() else []
def vmax(a,d,u,b,dt=DT):
    k=b*(a+dt)+u;delta=d-u*(a+dt)
    return 0.0 if delta<=0 else -k+math.sqrt(k*k+2*b*delta)
def actor_speed_series(grid):
    out=[];prev=None
    for g in grid:
        s=g.get('sim_s');a=g.get('actor_xy')
        if prev and a and s is not None and s>prev[0]:out.append((s,math.hypot(a[0]-prev[1][0],a[1]-prev[1][1])/(s-prev[0])))
        if a and s is not None:prev=(s,a)
    return out
def eq1(grid,edge,t0,t1,u_nom):
    n=int(round((t1-t0)/L.GRID_DT))
    if n<=0:return {}
    an=L.a_nav_grid(edge,t0,t1);v=L.hold_grid(grid,'v_mps',t0,t1);d=L.hold_grid(grid,'clearance_m',t0,t1)
    raw=L.hold_grid(grid,'raw_scan_age_wall_s',t0,t1)
    us=actor_speed_series(grid);ua=[];j=0;last=0.0
    for k in range(n):
        t=t0+k*L.GRID_DT
        while j<len(us) and us[j][0]<=t+1e-9:last=us[j][1];j+=1
        ua.append(last)
    out={'dist_m':round(sum(abs(x)*L.GRID_DT for x in v if x is not None),4)}
    out['P3_m']=round(sum(abs(x)*L.GRID_DT for x,a in zip(v,an) if x is not None and a is not None and a>L.NAV_STALE_S),4)
    out['P3_norm']=round(out['P3_m']/out['dist_m'],4) if out['dist_m']>0 else None
    for tag,ages in (('anav',an),('raw',raw)):
        for utag,useries in (('unom',[u_nom]*n),('uact',ua)):
            for b in BS:
                ts=ds=0.0
                for x,a,dd,uu in zip(v,ages,d,useries):
                    if x is None or dd is None or a is None or not math.isfinite(a):continue
                    if abs(x)>VMOVE and abs(x)>vmax(a,dd,uu,b):ts+=L.GRID_DT;ds+=abs(x)*L.GRID_DT
                out[f'viol_{tag}_{utag}_b{b}_s']=round(ts,2);out[f'viol_{tag}_{utag}_b{b}_m']=round(ds,4)
    cl=[x for x in d if x is not None];out['min_clearance_m']=round(min(cl),4) if cl else None
    return out
def encounter(grid,ped_start):
    if ped_start is None:return None
    for g in grid:
        s=g.get('sim_s');c=g.get('clearance_m');v=g.get('v_mps')
        if s is None or c is None or v is None or s<ped_start:continue
        if (c+0.22)-0.10<=braking_threshold(v):return {'t_enc':s,'v_enc':v,'a_nav_enc':g.get('a_nav_s'),'robot_xy_enc':g.get('robot_xy'),'actor_xy_enc':g.get('actor_xy')}
    return None
def contact_class(grid,first_contact):
    if not first_contact or first_contact.get('sim_s') is None:return None
    tc=first_contact['sim_s']
    vs=[g.get('v_mps') for g in grid if g.get('sim_s') is not None and tc-0.5-1e-9<=g['sim_s']<=tc+1e-9]
    return 'robot_moving' if any(v is not None and v>0.02 for v in vs) else 'pedestrian_initiated'
def actor_timing(td,cfg):
    """Arrival on the path (y >= 0.18) against plan (pedestrian start + (0.18-y0)/u)."""
    rows=jl(Path(td)/'E2_CROSSING_GROUND_TRUTH.jsonl')
    st=next((r['sim_s'] for r in rows if r.get('event')=='pedestrian_start'),None)
    arr=next((r['sim_s'] for r in rows if r.get('event')=='actor_commanded' and r.get('y',-9)>=0.18-1e-3),None)
    if st is None or arr is None:return None
    plan=st+(0.18-float(cfg['y0']))/float(cfg['speed']);return {'ped_start_sim_s':st,'arrival_sim_s':arr,'planned_sim_s':plan,'arrival_error_s':round(arr-plan,3)}
def run_measures(td,res):
    td=Path(td);grid=jl(td/'E3B_GRID.jsonl');edge=[(r['recv_sim_s'],r['stamp_s']) for r in jl(td/'E3B_EDGE_SCAN.jsonl')]
    cr=res.get('crossing') or {};cfg=cr.get('e5_cfg') or {}
    u=float(cfg.get('speed') or 0.08);t0=res.get('trigger_sim_s');g0=res.get('goal_sim_s');g1=res.get('terminal_sim_s')
    m={'speed':u,'ped_start_sim_s':cr.get('pedestrian_start_sim_s'),'max_actor_tracking_error_m':cr.get('max_actor_tracking_error_m'),
       'no_yield_close_events':cr.get('no_yield_close_events')}
    enc=encounter(grid,cr.get('pedestrian_start_sim_s'))
    m.update({'t_enc':enc['t_enc'] if enc else None,'enc_after_trigger_s':(enc['t_enc']-t0) if (enc and t0 is not None) else None,
              'v_enc':enc['v_enc'] if enc else None,'a_nav_enc':enc['a_nav_enc'] if enc else None,
              'robot_x_enc':enc['robot_xy_enc'][0] if (enc and enc['robot_xy_enc']) else None})
    fc=cr.get('first_actor_contact');m['contact_class']=contact_class(grid,fc);m['first_contact_sim_s']=fc.get('sim_s') if fc else None
    if t0 is not None:m.update({'W_'+k:v for k,v in eq1(grid,edge,t0,t0+L.WINDOW_S,u).items()})
    if g0 is not None and g1 is not None and g1>g0+1:m.update({'EP_'+k:v for k,v in eq1(grid,edge,g0,g1,u).items()})
    tm=actor_timing(td,cfg) if cfg else None
    if tm:m.update(tm)
    return m
