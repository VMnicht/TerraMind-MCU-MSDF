"""Read-only KF-GINS comparison views and a separate, explicit CSV export."""
from bisect import bisect_left
import csv
from math import ceil
from pathlib import Path
import tkinter as tk
from tkinter import ttk


def write_diagnostics_csv(result, path: Path) -> None:
    first = result.frames[0].mcu_ms
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("elapsed_s", "gps_week", "gps_tow_s", "imu_east_m", "imu_north_m", "imu_height_m",
                         "fused_antenna_east_m", "fused_antenna_north_m", "fused_antenna_height_m",
                         "fused_heading_deg", "gnss_vehicle_heading_deg", "gnss_heading_std_deg",
                         "heading_residual_deg"))
        for f in result.frames:
            writer.writerow(((f.mcu_ms - first) / 1000, f.gps_week, f.mcu_ms / 1000,
                             f.east_m, f.north_m, f.height_m, f.antenna_east_m, f.antenna_north_m,
                             f.antenna_height_m, f.heading_deg, f.gnss_vehicle_heading_deg,
                             f.gnss_heading_std_deg, f.heading_residual_deg))


class HeadingComparisonWindow:
    def __init__(self, app):
        self.app = app
        self.window = tk.Toplevel(app.root)
        self.window.title("KF-GINS 航向与观测残差对比")
        self.window.geometry("940x580")
        self.window.minsize(600, 400)
        ttk.Label(self.window, text="蓝：融合航向   绿：双天线换算车头航向   紫：融合 − 双天线（±180°）\n"
                  "按 GPS 时间对齐；相邻有效航向间隔 ≤150 ms 才插值，不外推。点击曲线区域定位主窗口回放。",
                  padding=8).pack(fill="x")
        self.canvas = tk.Canvas(self.window, bg="#f8fafc", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self.redraw())
        self.canvas.bind("<Button-1>", self._seek)
        self.cache_result = None
        self.series = None
        self.redraw()

    def exists(self):
        return bool(self.window.winfo_exists())

    def _load_series(self):
        result = self.app.result
        if result is self.cache_result:
            return
        self.cache_result = result
        first = result.frames[0].mcu_ms
        fused, reference, residual = [], [], []
        unwrapped = result.frames[0].heading_deg
        previous = unwrapped
        for f in result.frames:
            unwrapped += (f.heading_deg - previous + 180) % 360 - 180
            previous = f.heading_deg
            t = (f.mcu_ms - first) / 1000
            fused.append((t, unwrapped))
            e = f.heading_residual_deg
            reference.append((t, None if e is None else unwrapped - e))
            residual.append((t, e))
        self.series = fused, reference, residual

    def redraw(self):
        if not self.exists() or self.app.result is None:
            return
        self._load_series()
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100 or height < 100:
            return
        left, right = 70, width - 20
        duration = max(self.series[0][-1][0], .001)
        current = (self.app.result.frames[self.app.current_index].mcu_ms -
                   self.app.result.frames[0].mcu_ms) / 1000
        for row, (series, colors, title) in enumerate((
            (self.series[:2], ("#2563eb", "#16a34a"), "航向 / °（展开显示）"),
            ((self.series[2],), ("#7c3aed",), "航向残差 / °"),
        )):
            top, bottom = 25 + row * height / 2, (row + 1) * height / 2 - 35
            values = [v for s in series for _, v in s if v is not None]
            if row == 1:
                values.append(0)
            lo, hi = (min(values), max(values)) if values else (-1, 1)
            pad = max((hi - lo) * .08, 1)
            lo, hi = lo - pad, hi + pad
            xy = lambda t, v: (left + (right - left) * t / duration,
                                bottom - (bottom - top) * (v - lo) / (hi - lo))
            for k in range(5):
                v = lo + (hi - lo) * k / 4
                _, y = xy(0, v)
                canvas.create_line(left, y, right, y, fill="#e2e8f0")
                canvas.create_text(left - 8, y, text=f"{v:.1f}", anchor="e", fill="#64748b")
            for k in range(6):
                t = duration * k / 5
                x, _ = xy(t, lo)
                canvas.create_line(x, top, x, bottom, fill="#e2e8f0")
                canvas.create_text(x, bottom + 12, text=f"{t:.1f}s", fill="#64748b")
            canvas.create_text(left, top - 12, text=title, anchor="w")
            for s, color in zip(series, colors):
                # Split first, then decimate: never draw a line across a missing observation.
                segment = []
                def flush():
                    stride = max(1, ceil(len(segment) / 1200))
                    points = segment[::stride]
                    if segment and (not points or points[-1] != segment[-1]):
                        points.append(segment[-1])
                    if len(points) > 1:
                        canvas.create_line(*[c for t, v in points for c in xy(t, v)], fill=color, width=1.7)
                    elif points:
                        x, y = xy(*points[0])
                        canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="")
                    segment.clear()
                for t, v in s:
                    if v is None:
                        flush()
                    else:
                        segment.append((t, v))
                flush()
            if not any(v is not None for _, v in self.series[2]):
                canvas.create_text((left + right) / 2, (top + bottom) / 2,
                                   text="无有效双天线航向对照", fill="#92400e")
            x, _ = xy(current, lo)
            canvas.create_line(x, top, x, bottom, fill="#ea580c", dash=(4, 3))

    def _seek(self, event):
        if self.app.result is None:
            return
        fraction = min(1, max(0, (event.x - 70) / max(1, self.canvas.winfo_width() - 90)))
        times = self.app.frame_times
        target = times[0] + (times[-1] - times[0]) * fraction
        index = min(bisect_left(times, target), len(times) - 1)
        self.app.slider.set(index)
        self.app._scrub(str(index))
