#!/usr/bin/env python3
"""J4 speed level V (protocol section 3). Nothing frozen is edited; every change is made on a generated copy or at launch,
as in E8 (SCI_X2_HARD_ENCOUNTER_20261105, x2_stack.apply_local / x2_mode_gate / x2_guard):
  * DWB (generated controller YAML): FollowPath max_vel_x = max_speed_xy = V   (J1-J3: 0.18 from generate_movement_config);
  * mode gate: NORMAL cap = V (j4_mode_gate.py; SLOW 0.08 and STOP 0 unchanged)  (J1-J3: 0.15);
  * final hard guard: command cap MAX_LIN = max(0.18, V) (j4_guard.py; layer ON; all other constants frozen).
The safety layer's braking threshold is unchanged; it already depends on the measured speed."""
import hashlib, json
from pathlib import Path
import yaml
V_SET = (0.10, 0.15, 0.20, 0.22)      # 0.20 only as the pre-declared fallback for the top level (protocol section 4)
J1_DWB_CAP = 0.18


def _sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def level(spec):
    v = float((spec.get('j4') or {}).get('v_level', -1))
    if v not in V_SET:
        raise RuntimeError('J4_BAD_SPEED_LEVEL')
    return v


def apply_speed(movement, output, V):
    if V not in V_SET:
        raise RuntimeError('J4_BAD_SPEED_LEVEL')
    p = Path(movement['controller']); d = yaml.safe_load(p.read_text())
    fp = d['controller_server']['ros__parameters']['FollowPath']
    if abs(float(fp['max_vel_x']) - J1_DWB_CAP) > 1e-9 or abs(float(fp['max_speed_xy']) - J1_DWB_CAP) > 1e-9:
        raise RuntimeError('J4_UNEXPECTED_DWB_CAP')
    fp['max_vel_x'] = float(V); fp['max_speed_xy'] = float(V)
    q = Path(output) / 'j4_controller.yaml'; q.write_text(yaml.safe_dump(d, sort_keys=False))
    m = dict(movement); m['controller'] = str(q); m['sha256'] = dict(movement.get('sha256', {}), **{q.name: _sha(q)})
    m['j4'] = {'dwb_max_vel_x': float(V), 'dwb_max_speed_xy': float(V), 'mode_gate_normal_cap': float(V),
               'guard_max_lin': max(0.18, float(V))}
    (Path(output) / 'J4_SPEED.json').write_text(json.dumps(m['j4']) + '\n')
    return m
