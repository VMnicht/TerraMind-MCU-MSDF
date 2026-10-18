"""TerraMind 80-byte navigation v1 receiver. No GUI/serial dependencies."""
from __future__ import annotations
from dataclasses import dataclass
import math
import struct
import time

FRAME = struct.Struct('<2sBBIdddf3f3f4fHH')
VALID, FAULT, TIME_LOCKED, GNSS_RECENT, SIMULATED = 1, 2, 4, 8, 16

def crc16(data: bytes) -> int:
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc

@dataclass(frozen=True)
class Packet:
    status: int
    sequence: int
    state_time: float
    latitude: float
    longitude: float
    height: float
    velocity: tuple[float, float, float]
    attitude: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    age_ms: int

    @classmethod
    def decode(cls, data: bytes) -> 'Packet':
        f = FRAME.unpack(data)
        if f[0] != b'\xaa\x55' or f[1] != 1 or crc16(data[2:78]) != f[-1]:
            raise ValueError('Invalid navigation v1 frame')
        return cls(f[2], f[3], f[4], f[5], f[6], f[7], tuple(f[8:11]),
                   tuple(f[11:14]), tuple(f[14:18]), f[18])

    def validity(self) -> str:
        values = (self.state_time, self.latitude, self.longitude, self.height,
                  *self.velocity, *self.attitude, *self.quaternion)
        if not all(math.isfinite(x) for x in values): return '数值异常'
        if self.status & FAULT: return '导航故障'
        if not self.status & VALID: return '等待初始化 / 无有效结果'
        if self.state_time < 0 or abs(self.latitude) > math.pi / 2 or abs(self.longitude) > math.pi:
            return '坐标异常'
        if abs(math.sqrt(sum(q*q for q in self.quaternion)) - 1) > .001: return '四元数异常'
        if self.age_ms == 65535 or self.age_ms > 500: return '结果过期'
        return ''

class Parser:
    """Bounded resynchronizer; bad candidate discards one byte, never 80."""
    def __init__(self):
        self.buffer = bytearray()
        self.candidate_since: float | None = None
        self.frames = self.crc_errors = self.version_errors = self.discarded = self.timeouts = 0

    def feed(self, data: bytes = b'', now: float | None = None) -> list[Packet]:
        now = time.monotonic() if now is None else now
        packets: list[Packet] = []
        if self.candidate_since is not None and now-self.candidate_since >= .1:
            self.timeouts += 1; self.discarded += 1
            del self.buffer[0]; self.candidate_since = None
        # Process bounded pieces, retaining at most one incomplete candidate.
        chunks = [data[i:i+4096] for i in range(0, len(data), 4096)] or [b'']
        for chunk in chunks:
            self.buffer.extend(chunk)
            while self.buffer:
                at = self.buffer.find(b'\xaa\x55')
                if at < 0:
                    keep = 1 if self.buffer[-1] == 0xaa else 0
                    self.discarded += len(self.buffer)-keep
                    self.buffer[:] = b'\xaa' if keep else b''
                    self.candidate_since = None
                    break
                if at:
                    self.discarded += at; del self.buffer[:at]; self.candidate_since = None
                if len(self.buffer) < FRAME.size:
                    if self.candidate_since is None: self.candidate_since = now
                    if now-self.candidate_since < .1: break
                    self.timeouts += 1; self.discarded += 1
                    del self.buffer[0]; self.candidate_since = None
                    continue
                self.candidate_since = None
                if self.buffer[2] != 1:
                    self.version_errors += 1; self.discarded += 1; del self.buffer[0]; continue
                if crc16(self.buffer[2:78]) != int.from_bytes(self.buffer[78:80], 'little'):
                    self.crc_errors += 1; self.discarded += 1; del self.buffer[0]; continue
                packets.append(Packet.decode(bytes(self.buffer[:80])))
                self.frames += 1; del self.buffer[:80]
        return packets

class StreamState:
    def __init__(self):
        self.last_frame = self.last_valid = None
        self.sequence = self.state_time = self.measurement_time = None
        self.missing = self.duplicates = self.sessions = 0
        self.latest: Packet | None = None

    def accept(self, p: Packet, now: float) -> tuple[bool, bool]:
        """Return (new usable measurement, session reset). Invalid heartbeats
        prove transport health but cannot move the trajectory or freshness."""
        reset = False
        if self.sequence is not None:
            delta = (p.sequence-self.sequence) & 0xffffffff
            reset = delta >= 0x80000000 or (math.isfinite(p.state_time) and self.state_time is not None
                                           and p.state_time < self.state_time-1e-9)
            if not reset and delta > 1: self.missing += delta-1
            if delta == 0: self.duplicates += 1
        if reset:
            self.sessions += 1; self.last_valid = self.measurement_time = None
        duplicate_sequence = not reset and self.sequence == p.sequence
        self.sequence = p.sequence
        if math.isfinite(p.state_time): self.state_time = p.state_time
        self.latest = p; self.last_frame = now
        usable = not duplicate_sequence and not p.validity() and (
            self.measurement_time is None or p.state_time > self.measurement_time+1e-9)
        if usable:
            self.measurement_time = p.state_time; self.last_valid = now
        return usable, reset

    def health(self, now: float) -> str:
        if self.last_frame is None or now-self.last_frame > 3: return '连接超时 / 无完整帧'
        if self.latest is not None and self.latest.validity(): return self.latest.validity()
        if self.last_valid is None or now-self.last_valid > .1: return '导航流中断（超过 100 ms）'
        return '导航正常'

def demo_frame(sequence: int, t: float) -> bytes:
    """Clearly marked synthetic display data; never used for firmware input."""
    heading = t*.08
    q = (math.cos(heading/2), 0, 0, math.sin(heading/2))
    data = FRAME.pack(b'\xaa\x55', 1, VALID|TIME_LOCKED|GNSS_RECENT|SIMULATED|(4<<5), sequence & 0xffffffff,
        t, math.radians(30.5)+20*math.sin(heading)/6378137,
        math.radians(114)+20*(1-math.cos(heading))/(6378137*math.cos(math.radians(30.5))),
        20, 1.6*math.cos(heading), 1.6*math.sin(heading), 0, 0, 0, heading, *q, 200, 0)
    return data[:-2]+crc16(data[2:-2]).to_bytes(2,'little')
