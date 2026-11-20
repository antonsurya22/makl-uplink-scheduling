"""Declared edits that turn the frozen J3 j3_run_trial.py into j4_run_trial.py (applied in order; each must match once)."""
DOC_J4 = '''"""J4: ONE trial = the frozen J3 trial bring-up (j3_run_trial.py from SCI_J3_SWITCH_ABLATION_20261116, copied with the
declared edits in tests/j4_run_trial_edits.py) with exactly these changes:
  1. the C5.2 bridge starts through j4_bridge.py (J3 factory + MAKL_cap + v_hat in the decision log);
  2. the accepted configurations are B7, MAKL, MAKL_cap, DSW_1.0;
  3. speed level V = spec['j4']['v_level']: DWB max_vel_x = max_speed_xy = V on the generated controller YAML
     (j4_stack.apply_speed), mode-gate NORMAL cap = V (j4_mode_gate.py), hard-guard MAX_LIN = max(0.18, V) (j4_guard.py).
Everything else is J3 = J1 v2 (E7 bring-up, E5 trial process, AMCL tolerance 1.0 s, J1 ledger and decision files).
Usage: j4_run_trial.py TRIAL_SPEC_JSON OUTPUT_DIR"""'''


def doc_j3(src):
    a = src.index('"""J3: ONE trial'); b = src.index('OUTPUT_DIR"""', a) + len('OUTPUT_DIR"""')
    return src[a:b]


EDITS = [
    ("spec['j1'].get('config') in ('B7','MAKL','DSW_0.8','DSW_1.0','DSW_1.2'),'J3_CONFIG_MISSING')",
     "spec['j1'].get('config') in ('B7','MAKL','MAKL_cap','DSW_1.0'),'J4_CONFIG_MISSING')"),
    ("    req(isinstance(spec['j1'].get('plan'),dict),'J1_PLAN_MISSING')\n",
     "    req(isinstance(spec['j1'].get('plan'),dict),'J1_PLAN_MISSING')\n"
     "    import j4_stack as J4S;V=J4S.level(spec)   # J4: speed level\n"),
    ("        (output/'MOVEMENT_CHAIN_CONFIG.json').write_text(json.dumps(movement,indent=2)+'\\n')\n",
     "        movement=J4S.apply_speed(movement,output,V)   # J4: DWB max_vel_x = max_speed_xy = V\n"
     "        (output/'MOVEMENT_CHAIN_CONFIG.json').write_text(json.dumps(movement,indent=2)+'\\n')\n"),
    ("str(PJ/'j3_bridge.py')", "str(PJ/'j4_bridge.py')"),
    ("start('e2_mode_gate',['/usr/bin/python3','-u',str(ROOT/'e2_mode_gate.py')])",
     "start('e2_mode_gate',['/usr/bin/python3','-u',str(PJ/'j4_mode_gate.py'),str(V)])"),
    ("start('final_hard_guard',['/usr/bin/python3','-u',str(ROOT/'safety_chain.py')])",
     "start('final_hard_guard',['/usr/bin/python3','-u',str(PJ/'j4_guard.py'),str(V)])"),
    ("'J3_RUN_TRIAL_SETUP_BLOCKED'", "'J4_RUN_TRIAL_SETUP_BLOCKED'"),
]


def derive(src):
    s = src.replace(doc_j3(src), DOC_J4, 1)
    for a, b in EDITS:
        if s.count(a) != 1:
            raise ValueError('EDIT_NOT_UNIQUE ' + a)
        s = s.replace(a, b)
    return s
