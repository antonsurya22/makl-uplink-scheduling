"""J5b amendment A1: reading the BSSID.
In the J5b pilot PIL_N_Q2_B0 one of 23 BSSID samples came back empty (no BSSID, no frequency, no signal) 18.6 s after
the trigger, while every other sample, the samples before and after the replay, the data flow (257 of 257 messages at
the edge) and the kernel and wpa_supplicant logs (no disconnect event) show the same association throughout. The frozen
J5 reader turns any failed `iw dev wlan0 link` call (error or 5 s timeout) into "BSSID = none", which the validity rule
counts as a change. A1 separates the two cases:
  - "Not connected." from iw is a real disconnect: returned at once with bssid None (the replay stays invalid);
  - a failed read (error, timeout, or output without either line) is retried up to 2 more times, 0.3 s apart, with a
    2 s timeout each; if all 3 fail, bssid None is returned (invalid, as in J5).
Every sample records the attempts and the reason, so all retries are reported. The validity rule itself is unchanged."""
from __future__ import annotations
import re, time
import j5_sysops as S


def parse(o):
    d = {}
    m = re.search(r'Connected to ([0-9a-f:]{17})', o); d['bssid'] = m.group(1) if m else None
    m = re.search(r'freq: (\d+)', o); d['freq'] = int(m.group(1)) if m else None
    m = re.search(r'signal: (-?\d+)', o); d['signal_dbm'] = int(m.group(1)) if m else None
    m = re.search(r'tx bitrate: ([\d.]+)', o); d['tx_mbps'] = float(m.group(1)) if m else None
    return d


class SysOpsA1(S.SysOps):
    TRIES = 3; TIMEOUT_S = 2; GAP_S = 0.3

    def link_info(self):
        last = ''
        for k in range(1, self.TRIES + 1):
            rc, o = S.sh([S.IW, 'dev', self.iface, 'link'], timeout=self.TIMEOUT_S)
            if 'Not connected' in o:
                return {'bssid': None, 'freq': None, 'signal_dbm': None, 'tx_mbps': None, 'a1': 'not_connected', 'attempts': k}
            d = parse(o)
            if rc == 0 and d['bssid']:
                d['attempts'] = k
                return d
            last = o[-200:]
            time.sleep(self.GAP_S)
        return {'bssid': None, 'freq': None, 'signal_dbm': None, 'tx_mbps': None, 'a1': 'read_failed',
                'attempts': self.TRIES, 'last_output': last}
