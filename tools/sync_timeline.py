"""Read V2 TIM2 capture records and build a conditional GPS/PPS time map."""

from __future__ import annotations

from bisect import bisect_right
from collections import Counter
from dataclasses import dataclass
import csv
from pathlib import Path
from statistics import median
from typing import List, Optional

from monitor_protocol import unicore_crc32

GPS_WEEK_MS = 604_800_000
NOMINAL_TICKS_PER_SECOND = 4_000_000
MAX_GNSS_OUTPUT_LATENCY_MS = 300


@dataclass
class Pps:
    sequence: int
    mcu_ms: int
    ticks: int
    period_ticks: int


@dataclass
class Drdy:
    sequence: int
    mcu_ms: int
    ticks: int


@dataclass
class McuAnchor:
    pps_sequence: int
    gps_total_ms: int
    pps_ticks: int
    locked: bool


@dataclass
class ImuTiming:
    mcu_ms: int
    count: int
    drdy_sequence: int
    drdy_ticks: int
    uart_end_ticks: int


@dataclass
class GnssTiming:
    week: int
    tow_ms: int
    receive_ticks: int


@dataclass
class SyncTimeline:
    pps: List[Pps]
    drdy: List[Drdy]
    mcu_anchors: List[McuAnchor]
    imu: List[ImuTiming]
    gnss: List[GnssTiming]
    tick_rate: Optional[float]
    anchor_pps: Optional[Pps]
    anchor_gps_total_ms: Optional[int]
    candidate_count: int
    agreed_count: int
    agreed_latency_ms: List[float]

    @property
    def locked(self) -> bool:
        return self.anchor_pps is not None and self.anchor_gps_total_ms is not None and self.tick_rate is not None

    def gps_ms_to_local_ms(self, gps_total_ms: int) -> float:
        if not self.locked:
            raise ValueError("GPS/PPS second association unavailable")
        assert self.anchor_pps is not None and self.anchor_gps_total_ms is not None and self.tick_rate is not None
        return (self.anchor_pps.ticks +
                (gps_total_ms - self.anchor_gps_total_ms) * self.tick_rate / 1000.0) / 4000.0

    @staticmethod
    def tick_to_local_ms(ticks: int) -> float:
        return ticks / 4000.0

    def tick_to_gps_ms(self, ticks: int) -> float:
        if not self.locked:
            raise ValueError("GPS/PPS second association unavailable")
        assert self.anchor_pps is not None and self.anchor_gps_total_ms is not None and self.tick_rate is not None
        return self.anchor_gps_total_ms + (ticks - self.anchor_pps.ticks) * 1000.0 / self.tick_rate


def analyze_sync(path: Path) -> SyncTimeline:
    pps: List[Pps] = []
    drdy: List[Drdy] = []
    mcu_anchors: List[McuAnchor] = []
    imu: List[ImuTiming] = []
    gnss: List[GnssTiming] = []
    pending_gnss = None
    with path.open("r", encoding="ascii", errors="replace") as source:
        for line in source:
            line = line.strip()
            if not line:
                continue
            try:
                if line.startswith("P,"):
                    f = [int(x) for x in line.split(",")[1:]]
                    if len(f) == 5:
                        pps.append(Pps(f[0], f[1], (f[2] << 32) | f[3], f[4]))
                elif line.startswith("D,"):
                    f = [int(x) for x in line.split(",")[1:]]
                    if len(f) == 4:
                        drdy.append(Drdy(f[0], f[1], (f[2] << 32) | f[3]))
                elif line.startswith("A,"):
                    f = [int(x) for x in line.split(",")[1:]]
                    if len(f) == 10:
                        mcu_anchors.append(McuAnchor(f[1], f[2] * GPS_WEEK_MS + f[3],
                                                     (f[4] << 32) | f[5], bool(f[9])))
                elif line.startswith("T,"):
                    f = [int(x) for x in line.split(",")[1:]]
                    if len(f) == 7:
                        imu.append(ImuTiming(f[0], f[1], f[2],
                                             (f[3] << 32) | f[4], (f[5] << 32) | f[6]))
                elif line.startswith("N,"):
                    pending_gnss = None
                    _, _, raw = line.split(",", 2)
                    if raw.startswith("#BESTNAVA,"):
                        payload, check = raw.rsplit("*", 1)
                        if len(check) == 8 and unicore_crc32(payload[1:]) == int(check, 16):
                            header = payload[1:].split(";", 1)[0].split(",")
                            if len(header) == 10 and header[3] == "FINE":
                                pending_gnss = int(header[4]), int(header[5])
                elif line.startswith("R,"):
                    if pending_gnss is not None:
                        f = [int(x) for x in line.split(",")[1:]]
                        if len(f) == 3:
                            gnss.append(GnssTiming(*pending_gnss, (f[1] << 32) | f[2]))
                    pending_gnss = None
                elif not line.startswith(("D,", "I,", "S,", "H,", "V,")):
                    pending_gnss = None
            except (ValueError, IndexError):
                pending_gnss = None
    pps.sort(key=lambda p: p.ticks)
    periods = [p.period_ticks for p in pps if 3_500_000 <= p.period_ticks <= 4_500_000]
    rate = float(median(periods)) if periods else None
    anchor = None
    anchor_gps = None
    candidate_count = agreed_count = 0
    agreed_latency_ms: List[float] = []
    if rate is not None and pps:
        pps_ticks = [p.ticks for p in pps]
        candidates = []
        seen_pairs = set()
        for g in gnss:
            if g.tow_ms % 1000 != 0 or not 0 <= g.tow_ms < GPS_WEEK_MS:
                continue
            index = bisect_right(pps_ticks, g.receive_ticks) - 1
            if index < 0:
                continue
            elapsed_ms = (g.receive_ticks - pps[index].ticks) * 1000.0 / rate
            if 0 <= elapsed_ms <= MAX_GNSS_OUTPUT_LATENCY_MS:
                gps_total_ms = g.week * GPS_WEEK_MS + g.tow_ms
                pair = (pps[index].sequence, gps_total_ms)
                if pair not in seen_pairs:
                    candidates.append((pps[index], gps_total_ms, elapsed_ms))
                    seen_pairs.add(pair)
        candidate_count = len(candidates)
        if candidates:
            offsets = Counter(gps_ms // 1000 - p.sequence for p, gps_ms, _ in candidates)
            common_offset, agreed_count = offsets.most_common(1)[0]
            if agreed_count >= 3 and agreed_count * 5 >= candidate_count * 4:
                anchor, anchor_gps = next((p, ms) for p, ms, _ in candidates
                                          if ms // 1000 - p.sequence == common_offset)
                agreed_latency_ms = [delay for p, ms, delay in candidates
                                     if ms // 1000 - p.sequence == common_offset]
    return SyncTimeline(pps, drdy, mcu_anchors, imu, gnss, rate, anchor, anchor_gps,
                        candidate_count, agreed_count, agreed_latency_ms)


def export_timing_csv(timeline: SyncTimeline, path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("mcu_ms", "imu_count", "drdy_sequence", "drdy_ticks",
                         "uart_end_ticks", "uart_minus_drdy_us", "gps_week",
                         "gps_tow_ms", "gps_mapping_assumption"))
        for item in timeline.imu:
            delay_us = ""
            week = tow = ""
            if item.drdy_ticks and item.uart_end_ticks and timeline.tick_rate:
                delay_us = f"{(item.uart_end_ticks - item.drdy_ticks) * 1e6 / timeline.tick_rate:.2f}"
            if item.drdy_ticks and timeline.locked:
                gps_ms = timeline.tick_to_gps_ms(item.drdy_ticks)
                week = int(gps_ms // GPS_WEEK_MS)
                tow = f"{gps_ms - week * GPS_WEEK_MS:.3f}"
            writer.writerow((item.mcu_ms, item.count, item.drdy_sequence,
                             item.drdy_ticks or "", item.uart_end_ticks or "",
                             delay_us, week, tow,
                             "BESTNAVA output latency <800 ms" if week != "" else ""))
