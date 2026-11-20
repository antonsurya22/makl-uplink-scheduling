#!/usr/bin/env python3
"""J4 links = the frozen J3 links (j3_links.py from SCI_J3_SWITCH_ABLATION_20261116, imported unchanged: J1 v2 links +
the distance switch DSW_x) plus:
  1. the arm MAKL_cap = MAKL-FixedV. This is the frozen P0-J configuration 'MAKL_cap' (j_links.CONFIGS: jdisc 'makl',
     v_mode 'cap', v_cap 0.15). Same code path, same c_hat, peak test, m and beta_min as MAKL; the only difference is the
     speed passed to a*(v, d): the constant 0.15 m/s instead of v_hat = max(v_odom, v_cmd). It was a J0-only
     configuration; J4 only adds its name to the list of accepted J1 configurations.
  2. logging only: every decision record of a MAKL-type link (MAKL and MAKL_cap) also carries v_hat =
     max(v_odom, v_cmd) at the decision, so the analysis can check which speed was used and recompute the decision with
     the other speed. The decision itself is not changed."""
import sys
from pathlib import Path
J3 = Path.home() / 'ros2_ws' / 'SCI_J3_SWITCH_ABLATION_20261116'
if str(J3) not in sys.path:
    sys.path.insert(0, str(J3))
import j3_links as L3            # imports the frozen J1 v2 links (and through them P0-J)
JL = L3.JL
FIXV = 'MAKL_cap'
FIXV_SPEED = 0.15
J4_CONFIGS = ('B7', 'MAKL', FIXV, 'DSW_1.0')


class VHatSink(list):
    """Wraps the J1 decision sink: adds v_hat to each decision record, then writes it through the original sink."""
    def __init__(self, inner, link):
        super().__init__()
        self.inner = inner
        self.link = link

    @property
    def n(self):
        return self.inner.n

    def append(self, x):
        st = self.link.state
        self.inner.append(dict(x, v_hat=round(max(st['v_odom'], st['v_cmd']), 4)))


_MAKE_J3 = None


def make_link(config, **kw):
    L = _MAKE_J3(config, **kw)
    if getattr(L, 'jdisc', None) == 'makl' and getattr(L, 'dlog', None) is not None:
        L.dlog = VHatSink(L.dlog, L)
    return L


def install():
    """Patch the J1 link module before the J1 bridge is imported (the bridge looks these names up at run time)."""
    global _MAKE_J3
    L3.install()                                  # J3: DSW arms, make_link, is_makl
    if FIXV not in JL.J1_CONFIGS:
        JL.J1_CONFIGS = tuple(JL.J1_CONFIGS) + (FIXV,)
    if JL.J.CONFIGS[FIXV] != {'jdisc': 'makl', 'params': {'v_mode': 'cap'}} or JL.J.MAKL_DEFAULTS['v_cap'] != FIXV_SPEED:
        raise RuntimeError('J4_FROZEN_MAKL_CAP_CHANGED')
    _MAKE_J3 = JL.make_link
    JL.make_link = make_link
