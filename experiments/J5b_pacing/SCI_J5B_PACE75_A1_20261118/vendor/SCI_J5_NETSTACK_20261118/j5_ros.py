#!/usr/bin/env python3
"""ROS 2 glue for J5 (rclpy, CycloneDDS, domain 77). Data topics: best effort, keep-last 1 at the writer.
Control and file topics: reliable. Every request carries a sequence number; the reply is matched on it and a request
is re-sent every 5 s until answered (the edge answers a repeated sequence number from its cache)."""
from __future__ import annotations
import array, hashlib, json, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, UInt8MultiArray
import j5_common as C

QD_W = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                  durability=DurabilityPolicy.VOLATILE)
QD_R = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                  durability=DurabilityPolicy.VOLATILE)
QC = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE)


class RosTransport:
    """Sender side."""
    def __init__(self):
        rclpy.init()
        self.node = Node('j5_sender')
        self.pub = {s: self.node.create_publisher(UInt8MultiArray, t, QD_W) for s, t in C.TOPIC.items()}
        self.ctl = self.node.create_publisher(String, '/j5/ctl', QC)
        self.node.create_subscription(String, '/j5/ack', self._ack, QC)
        self.sess = '%x' % time.time_ns()          # a restarted sender must never get a cached reply of an earlier run
        self.seq = 0; self.waits = {}; self.lock = threading.Lock()
        self.ex = SingleThreadedExecutor(); self.ex.add_node(self.node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True); self.th.start()

    def _ack(self, msg):
        d = json.loads(msg.data)
        if d.get('sess') != self.sess:
            return
        w = self.waits.get(d.get('seq'))
        if w is not None:
            w[1] = d.get('reply'); w[0].set()

    def matched(self):
        return self.ctl.get_subscription_count() >= 1 and all(p.get_subscription_count() >= 1 for p in self.pub.values())

    def request(self, kind, body, timeout=30.0):
        t_end = time.monotonic() + timeout
        while not self.matched():
            if time.monotonic() > t_end:
                return None
            time.sleep(0.2)
        with self.lock:
            self.seq += 1; seq = self.seq
        ev = threading.Event(); self.waits[seq] = [ev, None]
        m = String(); m.data = json.dumps({'sess': self.sess, 'seq': seq, 'kind': kind, 'body': body}, default=str)
        while time.monotonic() < t_end:
            self.ctl.publish(m)
            if ev.wait(min(5.0, max(0.1, t_end - time.monotonic()))):
                return self.waits.pop(seq)[1]
        self.waits.pop(seq, None)
        return None

    def send_file(self, rid, name, text):
        return self.request('file', {'rid': rid, 'name': name, 'text': text, 'sha': hashlib.sha256(text.encode()).hexdigest()},
                            timeout=60)

    def publish_data(self, stream, b):
        m = UInt8MultiArray(); m.data = array.array('B', b)
        self.pub[stream].publish(m)

    def close(self):
        self.ex.shutdown(); self.node.destroy_node(); rclpy.shutdown()


class EdgeNode(Node):
    """Edge side: wraps EdgeCore."""
    def __init__(self, core):
        super().__init__('j5_edge')
        self.core = core; self.cache = {}
        for s, t in C.TOPIC.items():
            self.create_subscription(UInt8MultiArray, t, self._data, QD_R)
        self.create_subscription(String, '/j5/ctl', self._ctl, QC)
        self.ack = self.create_publisher(String, '/j5/ack', QC)

    def _data(self, msg):
        t = time.time_ns()                 # first action: edge receive time
        self.core.on_data(bytes(msg.data), t)

    def _ctl(self, msg):
        d = json.loads(msg.data); seq = d.get('seq'); key = (d.get('sess'), seq)
        if key in self.cache:
            rep = self.cache[key]
        else:
            k, b = d.get('kind'), d.get('body') or {}
            if k == 'file':
                rep = self.core.on_file(b['rid'], b['name'], b['text'], b['sha'])
            else:
                rep = self.core.on_ctl(k, b)
                print('J5_EDGE', k, json.dumps(b, default=str)[:120] if k != 'all_done' else '', json.dumps(rep)[:300], flush=True)
            self.cache[key] = rep
        m = String(); m.data = json.dumps({'sess': d.get('sess'), 'seq': seq, 'reply': rep}, default=str)
        self.ack.publish(m)
