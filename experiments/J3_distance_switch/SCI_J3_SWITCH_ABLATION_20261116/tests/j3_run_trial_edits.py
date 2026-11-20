"""Declared edits that turn the frozen J1 v2 j1_run_trial.py into j3_run_trial.py (applied in order; each must match once)."""
DOC_J3='''"""J3: ONE trial = the frozen J1 v2 trial bring-up (j1_run_trial.py from SCI_J1_SCHEDULERS_V2_20261113, copied with the
declared edits in tests/j3_run_trial_edits.py) with exactly these changes:
  1. the C5.2 bridge starts through j3_bridge.py (the frozen J1 launcher + the distance-switch arms);
  2. the accepted configurations are B7, MAKL, DSW_0.8, DSW_1.0, DSW_1.2.
Everything else is J1 v2 (E7 bring-up, E5 trial process, AMCL tolerance 1.0 s, J1 ledger and decision files).
Usage: j3_run_trial.py TRIAL_SPEC_JSON OUTPUT_DIR"""'''
def doc_j1(src):
    a=src.index('"""J1: ONE trial');b=src.index('OUTPUT_DIR"""',a)+len('OUTPUT_DIR"""');return src[a:b]
EDITS=[
 ("spec['j1'].get('config') in ('B0','B1','B2','B3','B4','B5','B7','MAKL'),'J1_CONFIG_MISSING')",
  "spec['j1'].get('config') in ('B7','MAKL','DSW_0.8','DSW_1.0','DSW_1.2'),'J3_CONFIG_MISSING')"),
 ("str(PJ/'j1_bridge.py')","str(PJ/'j3_bridge.py')"),
 ("'J1_RUN_TRIAL_SETUP_BLOCKED'","'J3_RUN_TRIAL_SETUP_BLOCKED'"),
]
def derive(src):
    s=src.replace(doc_j1(src),DOC_J3,1)
    for a,b in EDITS:
        if s.count(a)!=1:raise ValueError('EDIT_NOT_UNIQUE '+a)
        s=s.replace(a,b)
    return s
