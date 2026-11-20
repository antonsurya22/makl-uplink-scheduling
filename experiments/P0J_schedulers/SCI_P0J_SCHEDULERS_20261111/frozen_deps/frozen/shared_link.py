"""Standalone deterministic shared-link event simulator, Gate B.1 (not ROS-integrated).

Emulation time is INTEGER microseconds supplied by the caller. No sleeps or
wall-clock timers. This is an application-layer network *abstraction*, not an
implementation of RTPS/DDS. The ROS-side relays are not supplied here.
"""
from __future__ import annotations
from collections import deque, Counter
from dataclasses import dataclass, field
from typing import Optional
import hashlib
import heapq
import math


def _draw(seed: int, direction: str, label: str, tick: int, index: int) -> float:
    key = f'{seed}|{direction}|{label}|{tick}|{index}'.encode()
    return int.from_bytes(hashlib.blake2s(key, digest_size=8).digest(), 'big') / (1 << 64)


@dataclass
class Record:
    message_id: int
    stream: str
    payload_bytes: int
    created_us: int
    reliability: str
    total_packets: int
    status: str = 'queued'
    finished_us: Optional[int] = None
    received_packets: set[int] = field(default_factory=set)
    failed_packets: set[int] = field(default_factory=set)
    deadline_us: Optional[int] = None


@dataclass(frozen=True)
class TxPacket:
    message_id: int
    index: int
    wire_bytes: int
    is_repair: bool = False


class SharedLink:
    """One direction: bounded FIFO, serial serializer, delay/loss, optional repairs.

    Each message is atomically admitted; individual packets then share FIFO.
    Two independent instances model uplink and downlink. Loss draws are keyed
    to simulation-time 1ms slots and slot-local packet ordinals.
    """
    def __init__(self, *, name: str, seed: int, capacity_bps: int,
                 queue_limit_bytes: int, delay_us: int = 0, jitter_us: int = 0,
                 packet_loss: float = 0.0, mtu_payload_bytes: int = 1472,
                 packet_overhead_bytes: int = 28,
                 repair_wait_us: int = 100_000, retention_us: int = 1_000_000):
        if capacity_bps <= 0 or queue_limit_bytes <= 0 or mtu_payload_bytes <= 0:
            raise ValueError('Capacity/queue/MTU must be positive')
        if not 0 <= packet_loss <= 1 or min(delay_us, jitter_us, repair_wait_us, retention_us) < 0:
            raise ValueError('Invalid delay, loss, or repair parameters')
        self.name = name
        self.seed = seed
        self.capacity_bps = capacity_bps
        self.queue_limit_bytes = queue_limit_bytes
        self.delay_us = delay_us
        self.jitter_us = jitter_us
        self.packet_loss = packet_loss
        self.mtu = mtu_payload_bytes
        self.overhead = packet_overhead_bytes
        self.repair_wait_us = repair_wait_us
        self.retention_us = retention_us
        self.now_us = 0
        self.events: list[tuple] = []
        self.event_serial = 0
        self.next_message_id = 0
        self.waiting = deque()
        self.waiting_bytes = 0
        self.busy = False
        self.records: dict[int, Record] = {}
        self.stream_latest: dict[str, tuple[int, int]] = {}
        self.wire = Counter()
        self.tx_log = []
        self._slot_counts = Counter()

    def _schedule(self, when_us: int, event: str, data: object):
        if when_us < self.now_us:
            raise RuntimeError('Event in the past')
        self.event_serial += 1
        heapq.heappush(self.events, (when_us, self.event_serial, event, data))

    def _packets(self, mid: int, size: int):
        if size < 0:
            raise ValueError('Message size cannot be negative')
        size = max(1, size)
        out = []
        for i, offset in enumerate(range(0, size, self.mtu)):
            nbytes = min(self.mtu, size-offset) + self.overhead
            out.append(TxPacket(mid, i, nbytes))
        return out

    def offer(self, *, stream: str, payload_bytes: int, stamp_us: int,
              reliability: str = 'best_effort') -> int:
        if reliability not in ('best_effort', 'reliable'):
            raise ValueError('reliability must be best_effort or reliable')
        if stamp_us > self.now_us:
            raise ValueError('Cannot stamp message in the future')
        self.next_message_id += 1
        mid = self.next_message_id
        units = self._packets(mid, payload_bytes)
        record = Record(mid, stream, payload_bytes, stamp_us, reliability, len(units))
        self.records[mid] = record
        need = sum(p.wire_bytes for p in units)
        if need + self.waiting_bytes > self.queue_limit_bytes:
            self._finish(record, 'queue_drop')
        else:
            self.waiting.extend(units)
            self.waiting_bytes += need
            if reliability == 'reliable':
                record.deadline_us = self.now_us + self.retention_us
                self._schedule(record.deadline_us, 'expire', mid)
            self._start_next()
        return mid

    def _finish(self, record: Record, status: str):
        if record.status in ('delivered', 'queue_drop', 'channel_loss', 'expired'):
            return
        record.status = status
        record.finished_us = self.now_us
        if status == 'delivered':
            previous = self.stream_latest.get(record.stream)
            if previous is None or record.created_us > previous[0]:
                self.stream_latest[record.stream] = (record.created_us, self.now_us)

    def _start_next(self):
        if self.busy:
            return
        while self.waiting:
            p = self.waiting.popleft()
            self.waiting_bytes -= p.wire_bytes
            r = self.records[p.message_id]
            if r.status in ('queue_drop', 'expired', 'delivered') or (r.status == 'channel_loss' and p.is_repair):
                continue
            # ceil keeps wire rate at or below the configured capacity.
            duration_us = math.ceil(8 * p.wire_bytes * 1_000_000 / self.capacity_bps)
            self.busy = True
            self._schedule(self.now_us + duration_us, 'tx_done', (p, self.now_us))
            return

    def advance(self, target_us: int):
        """Advance the externally supplied SIMULATION clock (monotone)."""
        if target_us < self.now_us:
            raise ValueError('Simulation time cannot move backwards')
        while self.events and self.events[0][0] <= target_us:
            t, _, event, data = heapq.heappop(self.events)
            self.now_us = t
            if event == 'tx_done':
                p, tx_start_us = data
                self.busy = False
                self.wire['bytes_total'] += p.wire_bytes
                self.wire['bytes_repair' if p.is_repair else 'bytes_first'] += p.wire_bytes
                self.wire['packets_total'] += 1
                slot = t // 1000
                ordinal = self._slot_counts[slot]
                self._slot_counts[slot] += 1
                lost = _draw(self.seed, self.name, 'loss', slot, ordinal) < self.packet_loss
                self.tx_log.append((p.message_id, p.index, tx_start_us, t, p.wire_bytes, p.is_repair, lost))
                r = self.records[p.message_id]
                if r.status not in ('expired', 'delivered', 'queue_drop', 'channel_loss'):
                    if lost:
                        if r.reliability == 'best_effort':
                            r.failed_packets.add(p.index)
                            self._finish(r, 'channel_loss')
                        else:
                            self._schedule(t + self.repair_wait_us, 'repair', p)
                    else:
                        jitter_key = _draw(self.seed, self.name, 'jitter', slot, ordinal)
                        jitter = round((jitter_key * 2 - 1) * self.jitter_us)
                        arrival = t + max(0, self.delay_us + jitter)
                        self._schedule(arrival, 'arrival', (p.message_id, p.index))
                self._start_next()
            elif event == 'arrival':
                mid, index = data
                r = self.records[mid]
                if r.status not in ('expired', 'delivered', 'channel_loss', 'queue_drop'):
                    r.received_packets.add(index)
                    if len(r.received_packets) == r.total_packets:
                        self._finish(r, 'delivered')
            elif event == 'repair':
                p = data
                r = self.records[p.message_id]
                if r.status in ('queued',) and p.index not in r.received_packets:
                    if r.deadline_us is None or t < r.deadline_us:
                        retrans = TxPacket(p.message_id, p.index, p.wire_bytes, True)
                        if self.waiting_bytes + retrans.wire_bytes <= self.queue_limit_bytes:
                            self.waiting.append(retrans)
                            self.waiting_bytes += retrans.wire_bytes
                            self._start_next()
                        else:
                            self._finish(r, 'expired')
            elif event == 'expire':
                r = self.records[data]
                if r.status == 'queued':
                    self._finish(r, 'expired')
        self.now_us = target_us

    def outcomes(self):
        return dict(Counter(r.status for r in self.records.values()))

    def delivered_age_s(self, stream: str):
        return sorted((r.finished_us-r.created_us)/1e6 for r in self.records.values()
                      if r.stream == stream and r.status == 'delivered')

    def current_age_s(self, stream: str):
        data = self.stream_latest.get(stream)
        return None if data is None else (self.now_us - data[0]) / 1e6

    def ledger(self):
        # Only call after all offers and after draining the selected horizon.
        app = Counter()
        for r in self.records.values():
            app[r.status] += r.payload_bytes
        return {'application_payload_bytes_by_outcome': dict(app),
                'wire_bytes_total': self.wire['bytes_total'],
                'wire_bytes_first': self.wire['bytes_first'],
                'wire_bytes_repair': self.wire['bytes_repair'],
                'queue_bytes_now': self.waiting_bytes,
                'offered_payload_bytes': sum(r.payload_bytes for r in self.records.values())}
