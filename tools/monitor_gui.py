"""USART6 live monitor. Run: python tools/monitor_gui.py [--replay FILE]."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import queue
import re
import threading
import time
from math import degrees
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Optional

from monitor_protocol import MonitorModel
from capture_profile import CaptureProfile, RawCapture, load_capture


def corrected_delta_text(model: MonitorModel, profile: Optional[CaptureProfile], now: float) -> str:
    """Keep the row present and use one complete sample/timing pair only."""
    prefix = "去静止均值 Δθ °: "
    calibration = model.calibration
    if profile is None:
        return prefix + "— 配置无效"
    if calibration is None:
        return prefix + "— 等待板端标定记录"
    if calibration.mode != 3 or calibration.delta_ctrl != profile.delta_ctrl:
        return prefix + "— 标定模式/比例不匹配"
    pair = model.imu_display_pair
    if pair is None or pair[0].mode != 3:
        return prefix + "— 等待有效 I/T 配对"
    sample, dt, received_at = pair
    if now - received_at > 0.2:
        return prefix + "— 完整帧已过期"
    angle, _ = sample.increments(profile.delta_ctrl)
    corrected = [a - degrees(b) * dt for a, b in zip(angle, calibration.stationary_rate)]
    return prefix + " ".join(f"{v:+.6g}" for v in corrected) + "（最近完整帧）"


class InputWorker(threading.Thread):
    def __init__(self, output: queue.Queue, port: Optional[str] = None,
                 replay: Optional[Path] = None) -> None:
        super().__init__(daemon=True)
        self.output = output
        self.port = port
        self.replay = replay
        self.stop_event = threading.Event()
        self.capture_lock = threading.Lock()
        self.capture = None
        self.dropped = 0
        self.error = ""

    def start_capture(self, path: Path, profile: CaptureProfile) -> None:
        with self.capture_lock:
            if self.capture is not None:
                self.capture.close()
            self.capture = RawCapture(path, profile)

    def stop_capture(self) -> None:
        with self.capture_lock:
            if self.capture is not None:
                capture = self.capture
                self.capture = None
                try:
                    capture.close(self.error)
                except OSError as exc:
                    self.error = f"日志/配套配置保存失败：{exc}"

    def emit(self, line: str) -> None:
        try:
            self.output.put_nowait((time.monotonic(), line))
        except queue.Full:
            self.dropped += 1

    def run(self) -> None:
        try:
            if self.replay is not None:
                self._replay()
            else:
                self._serial()
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.stop_capture()

    def _replay(self) -> None:
        first_ms = None
        started = time.monotonic()
        with self.replay.open("r", encoding="ascii", errors="replace") as source:
            for line in source:
                if self.stop_event.is_set():
                    break
                match = re.match(r"^[INS],(\d+),", line)
                if match:
                    mcu_ms = int(match.group(1))
                    if first_ms is None:
                        first_ms = mcu_ms
                    target = started + (mcu_ms - first_ms) / 1000.0
                    while not self.stop_event.is_set() and time.monotonic() < target:
                        self.stop_event.wait(min(0.02, target - time.monotonic()))
                self.emit(line)

    def _serial(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("未安装 pyserial：请运行 python -m pip install pyserial") from exc
        pending = bytearray()
        with serial.Serial(self.port, 921600, timeout=0.1) as device:
            while not self.stop_event.is_set():
                block = device.read(4096)
                if not block:
                    continue
                with self.capture_lock:
                    if self.capture is not None:
                        self.capture.write(block)
                pending.extend(block)
                while b"\n" in pending:
                    row, _, remainder = pending.partition(b"\n")
                    pending = bytearray(remainder)
                    self.emit(row.decode("ascii", errors="replace"))
                if len(pending) > 2048:
                    pending.clear()
                    self.emit("!PC_LINE_OVERFLOW")


class Dashboard:
    def __init__(self, root: tk.Tk, replay: Optional[Path] = None) -> None:
        self.root = root
        root.title("TerraMind IMU / UM982 监视与采集")
        root.geometry("1180x900")
        root.minsize(1080, 850)
        self.model = MonitorModel()
        self.inbox: queue.Queue = queue.Queue(maxsize=4000)
        self.worker: Optional[InputWorker] = None
        self.last_worker_drop = 0
        self.source_text = tk.StringVar(value="未连接")
        self.port_text = tk.StringVar()
        self.capture_text = tk.StringVar(value="未采集")
        self.pos_limit_text = tk.StringVar(value="0.20")
        self.hdg_limit_text = tk.StringVar(value="2.0")
        self.baseline_text = tk.StringVar(value="")
        self.mode_text = tk.StringVar(value="3")
        self.delta_ctrl_text = tk.StringVar(value=f"0x{CaptureProfile().delta_ctrl:04X}")
        self.delta_confirmed = tk.BooleanVar(value=False)
        self.delta_ctrl_text.trace_add("write", lambda *_: self.delta_confirmed.set(False))
        self.capture_profile: Optional[CaptureProfile] = None
        self.banner_text = tk.StringVar(value="等待数据")
        self.reason_text = tk.StringVar(value="连接串口或回放日志后开始判断。")
        self.warning_text = tk.StringVar()
        self.imu_text = tk.StringVar()
        self.pos_text = tk.StringVar()
        self.hdg_text = tk.StringVar()
        self.link_text = tk.StringVar()
        self.calibration_text = tk.StringVar(value="上电静止标定期间板端保持串口静默；请保持静止至出现数据。")
        self._build()
        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.update)
        if replay is not None:
            root.after(200, lambda: self.open_replay(replay))

    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)
        bar = ttk.Frame(main)
        bar.pack(fill="x")
        ttk.Label(bar, text="串口").pack(side="left")
        self.ports = ttk.Combobox(bar, textvariable=self.port_text, width=14, state="readonly")
        self.ports.pack(side="left", padx=5)
        ttk.Button(bar, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(bar, text="连接", command=self.toggle_connect)
        self.connect_button.pack(side="left", padx=5)
        ttk.Button(bar, text="回放日志", command=self.choose_replay).pack(side="left", padx=5)
        ttk.Label(bar, textvariable=self.source_text).pack(side="right")

        banner = tk.Frame(main, bg="#374151", pady=12)
        banner.pack(fill="x", pady=(12, 0))
        self.banner = tk.Label(banner, textvariable=self.banner_text, bg="#374151",
                               fg="white", font=("Microsoft YaHei UI", 17, "bold"))
        self.banner.pack(anchor="w", padx=12)
        self.reasons = tk.Label(banner, textvariable=self.reason_text, bg="#374151",
                                fg="white", justify="left", anchor="w", wraplength=980)
        self.reasons.pack(fill="x", padx=12, pady=(4, 0))
        tk.Label(banner, textvariable=self.warning_text, bg="#374151", fg="#fcd34d",
                 justify="left", anchor="w", wraplength=980).pack(fill="x", padx=12, pady=(4, 0))

        settings = ttk.Frame(main)
        settings.pack(fill="x", pady=10)
        ttk.Label(settings, text="就绪阈值：水平标准差 ≤").pack(side="left")
        ttk.Entry(settings, textvariable=self.pos_limit_text, width=6).pack(side="left")
        ttk.Label(settings, text="m   航向标准差 ≤").pack(side="left")
        ttk.Entry(settings, textvariable=self.hdg_limit_text, width=6).pack(side="left")
        ttk.Label(settings, text="°   实测基线（可选）").pack(side="left")
        ttk.Entry(settings, textvariable=self.baseline_text, width=7).pack(side="left")
        ttk.Label(settings, text="m；高程标准差 ≤0.5 m，连续 5 s，基线波动 ≤0.1 m").pack(side="left", padx=5)

        profile_row = ttk.Frame(main)
        profile_row.pack(fill="x", pady=(0, 8))
        ttk.Label(profile_row, text="期望模式").pack(side="left")
        self.mode_choice = ttk.Combobox(profile_row, textvariable=self.mode_text,
                                       values=("3", "2"), width=3, state="readonly")
        self.mode_choice.pack(side="left", padx=5)
        ttk.Label(profile_row, text="3=增量 / 2=原始；200 Hz   DLT_CTRL").pack(side="left")
        self.delta_entry = ttk.Entry(profile_row, textvariable=self.delta_ctrl_text, width=9)
        self.delta_entry.pack(side="left", padx=5)
        self.delta_check = ttk.Checkbutton(profile_row, text="已核对传感器实际寄存器",
                                          variable=self.delta_confirmed)
        self.delta_check.pack(side="left")
        ttk.Label(profile_row, text="仅用于核对和换算，不向板子写配置", foreground="#6b7280").pack(side="left", padx=8)
        ttk.Label(main, textvariable=self.calibration_text, wraplength=1100,
                  foreground="#475569").pack(fill="x", pady=(0, 6))

        grid = ttk.Frame(main)
        grid.pack(fill="both", expand=True)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        grid.rowconfigure(0, weight=1)
        grid.rowconfigure(1, weight=1)
        for title, variable, row, col in (
            ("G365 IMU", self.imu_text, 0, 0),
            ("UM982 位置", self.pos_text, 0, 1),
            ("UM982 双天线航向", self.hdg_text, 1, 0),
            ("链路 / 时间", self.link_text, 1, 1),
        ):
            frame = ttk.LabelFrame(grid, text=title, padding=10)
            frame.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
            tk.Label(frame, textvariable=variable, anchor="nw", justify="left",
                     font=("Consolas", 11), padx=4, pady=4).pack(fill="both", expand=True)

        plots = ttk.Frame(main)
        plots.pack(fill="x", pady=(6, 0))
        self.pos_canvas = self._plot_frame(plots, "位置水平标准差（近 30 秒）", 0)
        self.hdg_canvas = self._plot_frame(plots, "航向标准差（近 30 秒）", 1)

        capture_bar = ttk.Frame(main)
        capture_bar.pack(fill="x", pady=(12, 0))
        self.capture_button = ttk.Button(capture_bar, text="开始保存原始日志", command=self.toggle_capture)
        self.capture_button.pack(side="left")
        ttk.Label(capture_bar, textvariable=self.capture_text).pack(side="left", padx=12)
        ttk.Label(capture_bar, text="保存按钮可用于未就绪时的故障诊断；就绪状态仅表示传感器采集条件。",
                  foreground="#6b7280").pack(side="right")

    @staticmethod
    def _plot_frame(parent: ttk.Frame, title: str, col: int) -> tk.Canvas:
        frame = ttk.LabelFrame(parent, text=title, padding=5)
        frame.grid(row=0, column=col, sticky="ew", padx=4)
        parent.columnconfigure(col, weight=1)
        canvas = tk.Canvas(frame, height=85, bg="#fafafa", highlightthickness=0)
        canvas.pack(fill="x")
        return canvas

    def refresh_ports(self) -> None:
        try:
            from serial.tools import list_ports
            names = [item.device for item in list_ports.comports()]
        except ImportError:
            names = []
        self.ports["values"] = names
        if self.port_text.get() not in names:
            self.port_text.set(names[0] if names else "")

    def _replace_worker(self, port: Optional[str] = None,
                        replay: Optional[Path] = None) -> None:
        self._stop_worker()
        self.model = MonitorModel()
        self.inbox = queue.Queue(maxsize=4000)
        self.last_worker_drop = 0
        self.worker = InputWorker(self.inbox, port=port, replay=replay)
        self.worker.start()
        self.source_text.set(f"回放 {replay.name}" if replay else f"{port} / 921600 8N1")
        self.connect_button.configure(text="断开")

    def _stop_worker(self) -> None:
        error = ""
        if self.worker is not None:
            self.worker.stop_event.set()
            self.worker.join(timeout=0.5)
            error = self.worker.error
            self.worker = None
        self.source_text.set("未连接")
        self.capture_text.set(error or "未采集")
        self.capture_button.configure(text="开始保存原始日志")
        self.connect_button.configure(text="连接")
        self._lock_profile(False)

    def toggle_connect(self) -> None:
        if self.worker is not None:
            self._stop_worker()
        elif self.port_text.get():
            self._replace_worker(port=self.port_text.get())
        else:
            messagebox.showinfo("串口", "未检测到串口；安装 pyserial 后点击刷新。")

    def choose_replay(self) -> None:
        name = filedialog.askopenfilename(filetypes=[("日志文本", "*.txt"), ("所有文件", "*.*")])
        if name:
            self.open_replay(Path(name))

    def open_replay(self, path: Path) -> None:
        try:
            metadata = load_capture(path, verify_log=True)
            profile = CaptureProfile(**metadata["profile"]) if metadata else CaptureProfile()
        except (OSError, ValueError) as exc:
            messagebox.showerror("采集配置错误", str(exc))
            return
        self.mode_text.set(str(profile.expected_mode))
        self.delta_ctrl_text.set(f"0x{profile.delta_ctrl:04X}")
        self.delta_confirmed.set(profile.delta_ctrl_confirmed)
        self._replace_worker(replay=path)

    def _profile(self) -> CaptureProfile:
        if self.capture_profile is not None:
            return self.capture_profile
        try:
            profile = CaptureProfile(int(self.mode_text.get()),
                                     int(self.delta_ctrl_text.get().strip(), 16),
                                     self.delta_confirmed.get())
            profile.validate()
            return profile
        except ValueError as exc:
            raise ValueError(f"采集配置无效（DLT_CTRL 按十六进制填写）：{exc}") from exc

    def _lock_profile(self, locked: bool) -> None:
        self.mode_choice.configure(state="disabled" if locked else "readonly")
        self.delta_entry.configure(state="disabled" if locked else "normal")
        self.delta_check.configure(state="disabled" if locked else "normal")
        if not locked:
            self.capture_profile = None

    def toggle_capture(self) -> None:
        if self.worker is None or self.worker.replay is not None or not self.worker.is_alive():
            messagebox.showinfo("采集", "请先连接实时串口。")
            return
        if self.worker.capture is not None:
            self.worker.stop_capture()
            self.capture_text.set(self.worker.error or "已停止保存（原始日志 + .capture.json）")
            self.capture_button.configure(text="开始保存原始日志")
            self._lock_profile(False)
            return
        default = f"terramind_{datetime.now():%Y%m%d_%H%M%S}.txt"
        name = filedialog.asksaveasfilename(defaultextension=".txt", initialfile=default,
                                             filetypes=[("日志文本", "*.txt")])
        if name:
            try:
                profile = self._profile()
                self.worker.start_capture(Path(name), profile)
            except (OSError, ValueError) as exc:
                messagebox.showerror("保存失败", str(exc))
                return
            self.capture_text.set(str(name))
            self.capture_profile = profile
            self._lock_profile(True)
            self.capture_button.configure(text="停止保存")

    @staticmethod
    def _age(now: float, then: float) -> str:
        return "无数据" if then < 0 else f"{max(0, now - then):.1f} s"

    def _render_plot(self, canvas: tk.Canvas, history, limit: float, now: float,
                     color: str) -> None:
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 50)
        data = [(t, value) for t, value in history if t >= now - 30]
        if not data:
            canvas.create_text(10, 12, text="等待数据", anchor="nw", fill="#6b7280")
            return
        ceiling = max(limit * 1.5, min(max(v for _, v in data), limit * 100), 0.01)
        y_limit = height - 8 - min(limit / ceiling, 1) * (height - 18)
        canvas.create_line(0, y_limit, width, y_limit, fill="#16a34a", dash=(3, 3))
        points = []
        for t, value in data:
            x = 6 + (t - (now - 30)) / 30 * (width - 12)
            y = height - 8 - min(value / ceiling, 1) * (height - 18)
            points.extend((x, y))
        if len(points) >= 4:
            canvas.create_line(*points, fill=color, width=2)
        canvas.create_text(width - 5, 5, text=f"末值 {data[-1][1]:.3f}", anchor="ne", fill=color)

    def update(self) -> None:
        now = time.monotonic()
        for _ in range(600):
            try:
                at, line = self.inbox.get_nowait()
            except queue.Empty:
                break
            self.model.feed(line, at)
        if self.worker is not None:
            if self.worker.dropped > self.last_worker_drop:
                self.model.record_errors += self.worker.dropped - self.last_worker_drop
                self.model.local_fault_at = now
                self.last_worker_drop = self.worker.dropped
            if not self.worker.is_alive() and self.worker.error:
                self.source_text.set(f"输入错误：{self.worker.error}")
                self.capture_text.set(f"保存已停止：{self.worker.error}")
                self._lock_profile(False)
                self.capture_button.configure(text="开始保存原始日志")
        profile = None
        try:
            pos_limit = float(self.pos_limit_text.get())
            hdg_limit = float(self.hdg_limit_text.get())
            baseline = float(self.baseline_text.get()) if self.baseline_text.get().strip() else None
            if pos_limit <= 0 or hdg_limit <= 0 or baseline is not None and baseline <= 0:
                raise ValueError()
            profile = self._profile()
            ready, issues, warnings = self.model.assess(now, pos_limit, hdg_limit, baseline, profile)
        except ValueError as exc:
            ready, issues, warnings = False, [str(exc) or "阈值必须为正数"], []
            pos_limit, hdg_limit = 0.2, 2.0
        if self.worker is not None and self.worker.error:
            ready = False
            issues.insert(0, self.worker.error)
        self.banner_text.set("● 可开始采集：传感器条件已满足" if ready else "● 尚未就绪")
        color = "#166534" if ready else "#991b1b"
        self.banner.configure(bg=color)
        self.reasons.configure(bg=color)
        for child in self.banner.master.winfo_children():
            if isinstance(child, tk.Label):
                child.configure(bg=color)
        self.reason_text.set("当前检查项均满足。" if ready else "原因：" + "；".join(issues[:5]))
        self.warning_text.set("；".join(warnings))
        imu = self.model.imu
        calibration = self.model.calibration
        if calibration:
            mean_text = " / ".join(f"{degrees(v):+.6f}" for v in calibration.stationary_rate)
            self.calibration_text.set(
                f"板端启动标定完成：{calibration.duration_us / 1e6:.3f} s / {calibration.samples} 帧，"
                f"静止均值 XYZ °/s：{mean_text}；I 日志保留原始值。")
        else:
            self.calibration_text.set("等待板端启动标定记录；上电保持静止至出现数据，旧固件无此记录。")
        if imu:
            axes = ("角速度 °/s: " + "  ".join(f"{x:+8.3f}" for x in imu.gyro_dps) + "\n"
                    "加速度 g : " + "  ".join(f"{x:+8.3f}" for x in imu.accel_g)) if imu.mode == 2 else "模式 3：原始增量值见日志"
            if imu.mode == 3:
                axes = "原始 Δθ: " + " ".join(map(str, imu.values[:3]))
                axes += "\n原始 Δv: " + " ".join(map(str, imu.values[3:]))
                if profile is not None:
                    angle, velocity = imu.increments(profile.delta_ctrl)
                    axes += "\nΔθ °: " + " ".join(f"{v:+.6g}" for v in angle)
                    axes += "\nΔv m/s: " + " ".join(f"{v:+.6g}" for v in velocity)
                    axes += f"\nDLT_CTRL=0x{profile.delta_ctrl:04X} " + (
                        "人工已核对" if profile.delta_ctrl_confirmed else "未核对，仅供预览")
                axes += "\n" + corrected_delta_text(self.model, profile, now)
            elif calibration and calibration.mode == 2:
                corrected = [a-degrees(b) for a,b in zip(imu.gyro_dps, calibration.stationary_rate)]
                axes += "\n去静止均值 °/s: " + " ".join(f"{v:+.6g}" for v in corrected)
            self.imu_text.set(f"更新距今: {self._age(now, self.model.imu_at)}\n"
                              f"模式: {imu.mode}    速率: {self.model.imu_rate(now):.1f} Hz\n"
                              f"COUNT: {imu.count}    FLAG: 0x{imu.flag:04X}\n"
                              f"COUNT 异常: {self.model.imu_count_gaps}\n{axes}")
        else:
            self.imu_text.set("等待 IMU 数据")
        pos = self.model.position
        if pos:
            self.pos_text.set(f"更新距今: {self._age(now, self.model.position_at)}\n"
                              f"状态: {pos.status}    时间: {pos.time_status}\n解类型: {pos.solution}\n"
                              f"经纬度: {pos.lat:.9f}, {pos.lon:.9f}\n"
                              f"高程: {pos.height_m:.3f} m    使用卫星: {pos.satellites}\n"
                              f"标准差 北/东/高: {pos.lat_std_m:.3f} / {pos.lon_std_m:.3f} / {pos.height_std_m:.3f} m\n"
                              f"水平速度: {pos.speed_mps:.3f} m/s    GPS TOW: {pos.tow_ms} ms")
        else:
            self.pos_text.set("等待 BESTNAVA 数据")
        hdg = self.model.heading
        if hdg:
            self.hdg_text.set(f"更新距今: {self._age(now, self.model.heading_at)}\n"
                              f"状态: {hdg.status}    时间: {hdg.time_status}\n解类型: {hdg.solution}\n"
                              f"航向: {hdg.heading_deg:.4f}°    俯仰: {hdg.pitch_deg:.4f}°\n"
                              f"航向标准差: {hdg.heading_std_deg:.4f}°\n"
                              f"基线长度: {hdg.baseline_m:.4f} m    GPS TOW: {hdg.tow_ms} ms")
        else:
            self.hdg_text.set("等待 UNIHEADINGA 数据")
        s = self.model.status
        drift = self.model.clock_drift_ms_per_s(now)
        self.link_text.set(
            f"S 更新距今: {self._age(now, self.model.status_at)}\n"
            f"固件丢记录: {s.get('tx_dropped', '—')}    DMA 丢字节: {s.get('tx_discarded_bytes', '—')}\n"
            f"IMU RX 溢出: {s.get('imu_rx_overflow', '—')}    GNSS RX 溢出: {s.get('gnss_rx_overflow', '—')}\n"
            f"接收/发送错误: {sum(s.get(k, 0) for k in ('imu_uart_errors', 'gnss_uart_errors', 'tx_uart_errors'))}\n"
            f"PC 记录损坏: {self.model.record_errors}    COUNT 异常: {self.model.imu_count_gaps}\n"
            f"队列峰值: {s.get('tx_queue_high_water', '—')} / 8192 B\n"
            f"MCU/GPS 漂移: {drift:+.2f} ms/s" if drift is not None else
            f"S 更新距今: {self._age(now, self.model.status_at)}\n"
            f"固件丢记录: {s.get('tx_dropped', '—')}    DMA 丢字节: {s.get('tx_discarded_bytes', '—')}\n"
            f"IMU RX 溢出: {s.get('imu_rx_overflow', '—')}    GNSS RX 溢出: {s.get('gnss_rx_overflow', '—')}\n"
            f"PC 记录损坏: {self.model.record_errors}    COUNT 异常: {self.model.imu_count_gaps}\n"
            f"队列峰值: {s.get('tx_queue_high_water', '—')} / 8192 B\n"
            "MCU/GPS 漂移: 估计中"
        )
        if self.model.version and self.model.version.startswith("V,2,"):
            h = self.model.sync_status
            period = self.model.pps_period_ticks
            self.link_text.set(self.link_text.get() +
                f"\nPPS: {self.model.pps_sequence} 次，周期 " +
                (f"{period} tick" if period is not None else "等待第二个沿") +
                f"    DRDY: {h.get('drdy_captures', '—')} 次" +
                f"\nIMU 未匹配 DRDY: {h.get('imu_unmatched', '—')}    " +
                f"捕获队列丢失: {h.get('drdy_queue_drops', '—')}/{h.get('pps_queue_drops', '—')}" +
                f"\nGPS/PPS 整秒配对: {'已连续建立' if self.model.anchor.get('locked') else '待建立'}")
        self._render_plot(self.pos_canvas, self.model.pos_std_history, pos_limit, now, "#2563eb")
        self._render_plot(self.hdg_canvas, self.model.hdg_std_history, hdg_limit, now, "#d97706")
        self.root.after(100, self.update)

    def close(self) -> None:
        self._stop_worker()
        self.root.destroy()


def analyze(path: Path) -> None:
    model = MonitorModel()
    metadata = load_capture(path, verify_log=True)
    profile = CaptureProfile(**metadata["profile"]) if metadata else CaptureProfile()
    last_at = 0.0
    for line in path.read_text(encoding="ascii", errors="replace").splitlines():
        match = re.match(r"^[INS],(\d+),", line)
        if match:
            last_at = int(match.group(1)) / 1000.0
        model.feed(line, last_at)
    ready, issues, warnings = model.assess(last_at, profile=profile)
    print(f"IMU {model.counts['I']} | BESTNAVA {model.counts['BESTNAVA']} | "
          f"UNIHEADINGA {model.counts['UNIHEADINGA']} | 坏记录 {model.record_errors}")
    print(f"位置 {model.position.status}/{model.position.solution}" if model.position else "无位置")
    print(f"航向 {model.heading.status}/{model.heading.solution}, "
          f"标准差 {model.heading.heading_std_deg:.3f}°" if model.heading else "无航向")
    print("可开始采集" if ready else "尚未就绪：" + "；".join(issues))
    for warning in warnings:
        print("提示：" + warning)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, help="以原始时序在界面回放日志")
    parser.add_argument("--analyze", type=Path, help="命令行分析日志，不启动界面")
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    root = tk.Tk()
    Dashboard(root, args.replay)
    root.mainloop()


if __name__ == "__main__":
    main()
