#!/usr/bin/env python3
"""E2 OFFLINE PROTOTYPE, not wired to a live bridge or motion controller.

Q0/Q1/Q2 are application-layer communication profiles, not DDS QoS.
The E2 pedestrian trigger must be acknowledged by the actual crossing process
and scoped to the live goal before this engine can become a runtime feature.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class TriggeredPlan:
    normal_mbps:float=3.0
    congest_mbps:float=.25
    recovery_mbps:float=3.0
    congest_after_trigger_s:float=0.0
    recover_after_trigger_s:float=8.0
    def __post_init__(self):
        vals=(self.normal_mbps,self.congest_mbps,self.recovery_mbps,
              self.congest_after_trigger_s,self.recover_after_trigger_s)
        if not all(type(x) in (float,int) and math.isfinite(x) for x in vals):raise ValueError('NONFINITE_PLAN')
        if not (3.0<=self.normal_mbps<=5 and .15<=self.congest_mbps<=.50 and 3<=self.recovery_mbps<=5):
            raise ValueError('INVALID_BANDWIDTH_BOUNDS')
        if not self.congest_after_trigger_s==0. or not 5<=self.recover_after_trigger_s<=20:
            raise ValueError('INVALID_TRIGGER_PHASE_BOUNDARIES')

class EventEpisode:
    def __init__(self,plan=None):
        self.plan=plan or TriggeredPlan();self.active=False;self.trigger_us=None;self.phase='idle';self.last_us=None
    def clock(self,now_us):
        if type(now_us) is not int or now_us<0 or self.last_us is not None and now_us<self.last_us:
            raise RuntimeError('INVALID_OR_ROLLBACK_SIM_CLOCK')
        self.last_us=now_us
    def begin(self,now_us,*,reset_verified=False,goal_accepted=False):
        self.clock(now_us)
        if self.active or reset_verified is not True or goal_accepted is not True:
            raise RuntimeError('UNATTESTED_EPISODE_BEGIN')
        self.active=True;self.trigger_us=None;self.phase='normal_start'
    def trigger(self,now_us,*,pedestrian_trigger_ack=False,goal_scope_match=False):
        self.clock(now_us)
        if not self.active:raise RuntimeError('TRIGGER_BEFORE_BEGIN')
        if self.trigger_us is not None:raise RuntimeError('DUPLICATE_TRIGGER')
        if pedestrian_trigger_ack is not True or goal_scope_match is not True:
            raise RuntimeError('TRIGGER_ACK_OR_SCOPE_MISSING')
        self.trigger_us=now_us
    def tick(self,now_us):
        self.clock(now_us)
        if not self.active or self.trigger_us is None:return []
        t0=self.trigger_us; t1=t0+round(self.plan.recover_after_trigger_s*1e6)
        result=[]
        if self.phase=='normal_start' and now_us>=t0:
            self.phase='congested';result.append(('congested',t0,self.plan.congest_mbps))
        if self.phase=='congested' and now_us>=t1:
            self.phase='recovery';result.append(('recovery',t1,self.plan.recovery_mbps))
        return result
    def finish(self,now_us,*,terminal_attested=False,stop_verified=False):
        self.clock(now_us)
        if not self.active or terminal_attested is not True or stop_verified is not True:
            raise RuntimeError('UNATTESTED_EPISODE_END')
        self.active=False;self.phase='idle'

class AdaptiveRule:
    """Decision-only example. Inputs MUST be verified raw on-board quantities, not 10-D index guesses."""
    def __init__(self):self.last=None;self.hold=0
    def act(self, distance_m, scan_age_s,mask):
        if len(mask)!=9 or not all(type(x) is bool for x in mask):raise ValueError('INVALID_MASK')
        if not (math.isfinite(distance_m) and 0<=distance_m<=100 and
                math.isfinite(scan_age_s) and scan_age_s>=0):raise ValueError('INVALID_SENSOR_QUANTITIES')
        near=distance_m<1.0;stale=scan_age_s>.25
        wanted=5 if near and stale else 2 if near else 1
        if self.hold<2 and self.last is not None:
            wanted=self.last
        # Hard shield overrides rule; never select an action outside the validated mask.
        if not mask[wanted]:
            if mask[8]:
                self.last=8;self.hold=0
                return 8,True
            raise RuntimeError('NO_SAFE_STOP_AVAILABLE')
        if wanted==self.last:self.hold+=1
        else:self.last=wanted;self.hold=1
        return wanted,False
