#!/usr/bin/env python3
"""J2 bridge launcher = the frozen J1 v2 launcher (j1_bridge.py from SCI_J1_SCHEDULERS_V2_20261113, imported unchanged:
E3-B joint C5.2 bridge + E5 race fix + J1 scheduler swap, capture stamps and MAKL inputs) with ONE addition: the uplink
that j1_links.make_link builds is wrapped by j2_impair.impair(link, SCI_J2_IMPAIR, SCI_J2_SEED) before any traffic.
Impairment statistics are printed every 20 scans ('J2_STATE') so every run shows that its impairment was active."""
import json, os, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
sys.path.insert(0, str(J1))
sys.path.insert(0, str(HERE))
import j1_links as JL
import j2_impair as I

_ORIG = JL.make_link


def _make_impaired(config, **kw):
    L = _ORIG(config, **kw)
    p = json.loads(os.environ['SCI_J2_IMPAIR'])
    seed = int(os.environ['SCI_J2_SEED'])
    cond = os.environ['SCI_J2_CONDITION']
    if p != I.CONDITIONS.get(cond):
        raise RuntimeError('J2_IMPAIR_DOES_NOT_MATCH_CONDITION')
    I.impair(L, p, seed)
    print('J2_IMPAIR condition=%s seed=%d params=%s link=%s' % (cond, seed, json.dumps(p, sort_keys=True),
                                                               type(L).__name__), flush=True)
    return L


JL.make_link = _make_impaired          # j1_bridge looks it up at construction time
import j1_bridge as JB                 # sets JB.M.JointC52Bridge = race_safe(J1 launcher)


def make_j2(base):
    class J2Bridge(base):
        def __init__(self, args):
            super().__init__(args)
            if not hasattr(self.link, 'j2'):
                raise RuntimeError('J2_LINK_NOT_IMPAIRED')
            self.j2_scans = 0

        def _offer_scan(self, msg):
            super()._offer_scan(msg)
            self.j2_scans += 1
            if self.j2_scans % 20 == 0:
                print('J2_STATE scans=%d %s' % (self.j2_scans, json.dumps(self.link.j2_summary(), sort_keys=True)),
                      flush=True)

    J2Bridge.__name__ = J2Bridge.__qualname__ = 'J2' + base.__name__
    return J2Bridge


JB.M.JointC52Bridge = make_j2(JB.M.JointC52Bridge)
if __name__ == '__main__':
    print('J2_BRIDGE class', JB.M.JointC52Bridge.__name__, flush=True)
    JB.M.main()
