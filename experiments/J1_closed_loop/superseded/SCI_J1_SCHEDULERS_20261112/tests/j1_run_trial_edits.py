"""Declared edits that turn the E7 package's e7_run_trial.py into j1_run_trial.py (applied in order; each must match once)."""
DOC_J1='''"""J1: ONE trial = the E7 (article E6) trial bring-up (e7_run_trial.py from the hash-checked E7 package, copied with the
declared edits in tests/j1_run_trial_edits.py) with exactly these changes:
  1. the C5.2 bridge starts through j1_bridge.py (the E7 launcher pattern with the J1 uplink scheduler B0-B7/MAKL);
  2. the bridge receives SCI_J1_CONFIG, SCI_J1_LEDGER and SCI_J1_DECISIONS (files in the trial directory);
  3. the capacity schedule is spec['j1']['plan'] (= e3b_policy_logic.plan(level) with the event duration 8 or 25 s;
     for C0 the unchanged plan(C0)).
The trial process is the E5 package's e5_trial.py; AMCL tolerance 1.0 s as P0/E5; everything else is E3-B.
Usage: j1_run_trial.py TRIAL_SPEC_JSON OUTPUT_DIR"""'''
def doc_e7(src):
    a=src.index('"""E7: ONE trial');b=src.index('OUTPUT_DIR"""',a)+len('OUTPUT_DIR"""');return src[a:b]
EDITS=[
 ("P7=Path(__file__).resolve().parent;P5=Path.home()/'ros2_ws/SCI_E5_PED_SPEED_20261101'",
  "PJ=Path(__file__).resolve().parent;P7=Path.home()/'ros2_ws/SCI_E7_QUEUE_DISCIPLINE_20261102';P5=Path.home()/'ros2_ws/SCI_E5_PED_SPEED_20261101'"),
 ("    req(isinstance(spec.get('e7'),dict) and spec['e7'].get('config') in ('K0','K1','K2','K3'),'E7_CONFIG_MISSING')\n",
  "    req(isinstance(spec.get('j1'),dict) and spec['j1'].get('config') in ('B0','B1','B2','B3','B4','B5','B7','MAKL'),'J1_CONFIG_MISSING')\n"
  "    req(isinstance(spec['j1'].get('plan'),dict),'J1_PLAN_MISSING')\n"),
 ("SCI_D3_SCHEDULES_JSON=json.dumps([L3.plan(spec['level'])]),\n              SCI_E7_QDISC=spec['e7']['config'],SCI_E7_LEDGER=str(output/'E7_LEDGER.jsonl'))",
  "SCI_D3_SCHEDULES_JSON=json.dumps([spec['j1']['plan']]),\n              SCI_J1_CONFIG=spec['j1']['config'],SCI_J1_LEDGER=str(output/'J1_LEDGER.jsonl'),SCI_J1_DECISIONS=str(output/'J1_DECISIONS.jsonl'))"),
 ("str(P7/'e7_bridge.py')","str(PJ/'j1_bridge.py')"),
 ("'E7_RUN_TRIAL_SETUP_BLOCKED'","'J1_RUN_TRIAL_SETUP_BLOCKED'"),
]
def derive(src):
    s=src.replace(doc_e7(src),DOC_J1,1)
    for a,b in EDITS:
        if s.count(a)!=1:raise ValueError('EDIT_NOT_UNIQUE '+a)
        s=s.replace(a,b)
    return s
