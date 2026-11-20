#!/usr/bin/env python3
"""System commands used by the J5 sender on the Raspberry Pi (iw, tc, chronyc, ping, iperf3).
tc and iw run through `sudo -n` (JALANKAN_J5_ROBOT.sh installs a sudoers rule for these two programs only)."""
from __future__ import annotations
import json, os, re, subprocess
from pathlib import Path

IFACE = 'wlan0'
TC = '/usr/sbin/tc'; IW = '/usr/sbin/iw'


def sh(cmd, timeout=20):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
        return p.returncode, p.stdout
    except Exception as ex:
        return 99, repr(ex)


class SysOps:
    def __init__(self, iface=IFACE):
        self.iface = iface; self.tc_log = []

    def versions(self):
        rc, k = sh(['uname', '-r'])
        rc, p = sh(['dpkg-query', '-W', '-f=${Package} ${Version}\n', 'ros-humble-rclpy', 'ros-humble-rmw-cyclonedds-cpp',
                    'ros-humble-cyclonedds', 'chrony', 'iproute2', 'iw', 'iperf3'])
        return {'kernel': k.strip(), 'packages': sorted(l for l in p.splitlines() if l.strip())}

    def iface_ip(self):
        rc, o = sh(['ip', '-4', '-o', 'addr', 'show', 'dev', self.iface])
        m = re.search(r'inet (\d+\.\d+\.\d+\.\d+)/', o)
        return m.group(1) if m else None

    def link_info(self):
        rc, o = sh([IW, 'dev', self.iface, 'link'], timeout=5)
        d = {}
        m = re.search(r'Connected to ([0-9a-f:]{17})', o); d['bssid'] = m.group(1) if m else None
        m = re.search(r'freq: (\d+)', o); d['freq'] = int(m.group(1)) if m else None
        m = re.search(r'signal: (-?\d+)', o); d['signal_dbm'] = int(m.group(1)) if m else None
        m = re.search(r'tx bitrate: ([\d.]+)', o); d['tx_mbps'] = float(m.group(1)) if m else None
        return d

    def power_save_off(self):
        sh(['sudo', '-n', IW, 'dev', self.iface, 'set', 'power_save', 'off'])
        rc, o = sh([IW, 'dev', self.iface, 'get', 'power_save'])
        return 'off' in o.lower()

    def throttled(self):
        rc, o = sh(['vcgencmd', 'get_throttled'], timeout=5)
        m = re.search(r'0x([0-9a-fA-F]+)', o) if rc == 0 else None
        if not m:
            p = Path('/sys/devices/platform/soc/soc:firmware/get_throttled')
            if p.is_file():
                try:
                    return int(p.read_text().strip(), 16) & 0xF
                except Exception:
                    return None
            return None
        return int(m.group(1), 16) & 0xF

    def clock(self):
        rc, o = sh(['chronyc', '-n', '-c', 'tracking'], timeout=5)
        f = o.strip().split(',')
        if rc != 0 or len(f) < 6:
            return None
        try:
            return {'ref': f[1], 'stratum': int(f[2]), 'offset_s': float(f[4]), 'last_offset_s': float(f[5]),
                    'rms_offset_s': float(f[6]), 'root_disp_s': float(f[11]) if len(f) > 11 else None}
        except ValueError:
            return None

    def env(self):
        return {k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'ROS_LOCALHOST_ONLY')}

    def ping(self, ip, count=50):
        rc, o = sh(['ping', '-i', '0.2', '-s', '2972', '-c', str(count), '-W', '2', ip], timeout=count * 0.2 + 20)
        ts = [float(x) for x in re.findall(r'time=([\d.]+)', o)]
        m = re.search(r'([\d.]+)% packet loss', o)
        if not m:
            return None
        ts.sort()
        return {'n': len(ts), 'loss_pct': float(m.group(1)), 'p50_ms': ts[len(ts) // 2] if ts else 999.0,
                'p99_ms': ts[min(len(ts) - 1, int(0.99 * len(ts)))] if ts else 999.0, 'max_ms': ts[-1] if ts else 999.0}

    def iperf(self, ip):
        rc, o = sh(['iperf3', '-c', ip, '-u', '-b', '2M', '-t', '10', '-J'], timeout=40)
        try:
            j = json.loads(o); s = j['end']['sum']
            return {'send_bps': s.get('bits_per_second'), 'lost_pct': s.get('lost_percent'),
                    'recv_bps': s.get('bits_per_second', 0) * (1 - (s.get('lost_percent') or 0) / 100), 'jitter_ms': s.get('jitter_ms')}
        except Exception:
            return {'error': o[-300:]}

    def _tc(self, verb, rate_bps, wait=True):
        cmd = ['sudo', '-n', TC, 'qdisc', verb, 'dev', self.iface, 'root', 'tbf', 'rate', f'{int(rate_bps)}bit', 'burst', '1600',
               'limit', '768000']
        if wait:
            rc, o = sh(cmd); self.tc_log.append((verb, rate_bps, rc, o.strip()[:200]))
            if rc != 0:
                raise RuntimeError('J5_TC_FAILED ' + o)
        else:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.tc_log.append((verb, rate_bps, None, 'async'))

    def tc_replace(self, rate_bps):
        self._tc('replace', rate_bps)

    def tc_change(self, rate_bps):
        self._tc('change', rate_bps, wait=False)

    def tc_delete(self):
        sh(['sudo', '-n', TC, 'qdisc', 'del', 'dev', self.iface, 'root'])

    def qdisc_stats(self):
        rc, o = sh([TC, '-s', 'qdisc', 'show', 'dev', self.iface], timeout=3)
        if rc != 0:
            return None
        lines = o.splitlines()
        root = next((i for i, l in enumerate(lines) if ' root ' in l or l.split()[:1] == ['qdisc'] and 'root' in l), None)
        if root is None:
            return None
        blk = '\n'.join(lines[root:root + 4])
        m = re.search(r'Sent (\d+) bytes (\d+) pkt \(dropped (\d+), overlimits (\d+)', blk)
        b = re.search(r'backlog (\d+)([KMG]?)b (\d+)p', blk)
        mult = {'': 1, 'K': 1000, 'M': 1_000_000, 'G': 1_000_000_000}
        return {'kind': lines[root].split()[1], 'sent_bytes': int(m.group(1)) if m else None,
                'sent_pkts': int(m.group(2)) if m else None, 'dropped': int(m.group(3)) if m else None,
                'backlog_bytes': int(b.group(1)) * mult[b.group(2)] if b else None, 'backlog_pkts': int(b.group(3)) if b else None}
