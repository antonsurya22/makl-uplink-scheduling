"""Declared edits that turn the frozen J1 v2 j1_run_trial.py into j2_run_trial.py (applied in order; each must match once)."""
DOC_J2='''"""J2: ONE trial = the frozen J1 v2 trial bring-up (j1_run_trial.py from SCI_J1_SCHEDULERS_V2_20261113, copied with the
declared edits in tests/j2_run_trial_edits.py) with exactly these changes:
  1. the C5.2 bridge starts through j2_bridge.py (the frozen J1 launcher + the J2 link impairment);
  2. the bridge also receives SCI_J2_CONDITION, SCI_J2_IMPAIR (JSON) and SCI_J2_SEED from spec['j2'].
Everything else is J1 v2 (E7 bring-up, E5 trial process, AMCL tolerance 1.0 s, J1 ledger and MAKL decision files).
Usage: j2_run_trial.py TRIAL_SPEC_JSON OUTPUT_DIR"""'''
def doc_j1(src):
    a=src.index('"""J1: ONE trial');b=src.index('OUTPUT_DIR"""',a)+len('OUTPUT_DIR"""');return src[a:b]
EDITS=[
 ("    req(isinstance(spec['j1'].get('plan'),dict),'J1_PLAN_MISSING')\n",
  "    req(isinstance(spec['j1'].get('plan'),dict),'J1_PLAN_MISSING')\n"
  "    req(isinstance(spec.get('j2'),dict) and isinstance(spec['j2'].get('impair'),dict) and isinstance(spec['j2'].get('seed'),int),'J2_IMPAIR_MISSING')\n"),
 ("SCI_J1_DECISIONS=str(output/'J1_DECISIONS.jsonl'))",
  "SCI_J1_DECISIONS=str(output/'J1_DECISIONS.jsonl'),\n              SCI_J2_CONDITION=spec['j2']['condition'],SCI_J2_IMPAIR=json.dumps(spec['j2']['impair'],sort_keys=True),SCI_J2_SEED=str(spec['j2']['seed']))"),
 ("str(PJ/'j1_bridge.py')","str(PJ/'j2_bridge.py')"),
 ("'J1_RUN_TRIAL_SETUP_BLOCKED'","'J2_RUN_TRIAL_SETUP_BLOCKED'"),
]
def derive(src):
    s=src.replace(doc_j1(src),DOC_J2,1)
    for a,b in EDITS:
        if s.count(a)!=1:raise ValueError('EDIT_NOT_UNIQUE '+a)
        s=s.replace(a,b)
    return s
