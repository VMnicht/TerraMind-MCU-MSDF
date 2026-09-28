"""Inspect V2 TIM2 DRDY/PPS sync logs; optionally export IMU sample GPS times."""

from __future__ import annotations

import argparse
from pathlib import Path
from statistics import median

from sync_timeline import (MAX_GNSS_OUTPUT_LATENCY_MS,
                           NOMINAL_TICKS_PER_SECOND, analyze_sync, export_timing_csv)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="USART6 原始文本日志")
    parser.add_argument("--csv", type=Path, help="导出 IMU/PPS 对齐后的时间表")
    args = parser.parse_args()
    timeline = analyze_sync(args.log)
    matched = [i for i in timeline.imu if i.drdy_sequence and i.drdy_ticks]
    print(f"PPS {len(timeline.pps)} 个；DRDY 捕获 {len(timeline.drdy)} 个；IMU 时间记录 {len(timeline.imu)} 条，DRDY 匹配 {len(matched)} 条；BESTNAVA 接收时间 {len(timeline.gnss)} 条")
    if timeline.tick_rate is not None:
        ppm = (timeline.tick_rate / NOMINAL_TICKS_PER_SECOND - 1) * 1e6
        print(f"PPS 实测 TIM2 周期中位数 {timeline.tick_rate:.0f} tick/s，相对标称 {ppm:+.1f} ppm")
        drdy_intervals = [(b.ticks - a.ticks) * 1000 / timeline.tick_rate
                          for a, b in zip(timeline.drdy, timeline.drdy[1:])
                          if b.sequence == a.sequence + 1]
        if drdy_intervals:
            print(f"DRDY 周期中位数 {median(drdy_intervals):.3f} ms；范围 {min(drdy_intervals):.3f}–{max(drdy_intervals):.3f} ms")
    if matched and timeline.tick_rate:
        delays = [(i.uart_end_ticks - i.drdy_ticks) * 1000 / timeline.tick_rate
                  for i in matched if i.uart_end_ticks >= i.drdy_ticks]
        if delays:
            print(f"UART 帧尾相对 DRDY 延迟中位数 {median(delays):.3f} ms；范围 {min(delays):.3f}–{max(delays):.3f} ms")
    print(f"GPS 整秒候选 {timeline.candidate_count} 个，其中一致 {timeline.agreed_count} 个")
    locked_anchors = [anchor for anchor in timeline.mcu_anchors if anchor.locked]
    print(f"MCU A 配对记录 {len(timeline.mcu_anchors)} 条，其中锁定 {len(locked_anchors)} 条")
    if timeline.locked:
        print(f"GPS/PPS 整秒已条件配对：假设 BESTNAVA 对应历元的串口末字节在 PPS 后 {MAX_GNSS_OUTPUT_LATENCY_MS} ms 内到达。")
        if timeline.agreed_latency_ms:
            print(f"整秒 BESTNAVA 末字节相对 PPS 延迟中位数 {median(timeline.agreed_latency_ms):.3f} ms")
        assert timeline.anchor_pps is not None and timeline.anchor_gps_total_ms is not None
        offset = timeline.anchor_gps_total_ms // 1000 - timeline.anchor_pps.sequence
        disagreements = sum(a.gps_total_ms // 1000 - a.pps_sequence != offset
                            for a in locked_anchors)
        if disagreements:
            print(f"警告：{disagreements} 条 MCU 锁定配对与离线 GPS 整秒关系不一致")
        print("请核实 UM982 输出延迟；仅靠这些消息无法排除整整一秒的归属错误。")
    else:
        print("GPS 整秒尚未可靠配对；PPS 和 DRDY 仍共享 TIM2 硬件时基。")
    if args.csv:
        export_timing_csv(timeline, args.csv)
        print(f"已写入 {args.csv}")


if __name__ == "__main__":
    main()
