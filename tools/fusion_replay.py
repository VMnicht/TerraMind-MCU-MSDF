"""Visual 2-D offline IMU/UM982 fusion replay; no height is displayed."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import asdict
import json
from math import ceil, cos, degrees, floor, hypot, log10, radians, sin
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Optional

from fusion2d import FusionResult, Geometry, ImuInputOptions, run_fusion
from capture_profile import load_capture
from startup_calibration import load_startup_calibration


LABELS = (
    ("lever_x_m", "主天线相对 IMU  +X / m"),
    ("lever_y_m", "主天线相对 IMU  +Y / m"),
    ("heading_offset_deg", "GNSS 航向 → IMU +X 顺时针角 / °"),
    ("gyro_heading_sign", "陀螺 Z 对顺时针航向的符号 ±1"),
    ("imu_y_right_sign", "IMU +Y 指向右侧填 +1，否则 -1"),
    ("gnss_delay_ms", "旧版日志 GNSS 历元提前量 / ms"),
)


class ReplayApp:
    def __init__(self, root: tk.Tk, initial_log: Optional[Path] = None) -> None:
        self.root = root
        root.title("TerraMind 二维组合导航离线回放")
        root.geometry("1160x900")
        root.minsize(880, 680)
        self.log_path = tk.StringVar(value=str(initial_log) if initial_log else "")
        # Same vehicle as the KF-GINS GUI, expressed in sensor X/Y coordinates.
        defaults = Geometry(lever_x_m=0.15, lever_y_m=0.0, heading_offset_deg=180.0,
                            gyro_heading_sign=1, imu_y_right_sign=1, gnss_delay_ms=0.0)
        self.fields: Dict[str, tk.StringVar] = {
            key: tk.StringVar(value=str(value)) for key, value in asdict(defaults).items()
        }
        self.speed = tk.StringVar(value="1×")
        self.calibrate_static_imu = tk.BooleanVar(value=False)
        self.delta_ctrl = tk.StringVar(value="")
        self.apply_delta_startup = tk.BooleanVar(value=True)
        self.imu_input_text = tk.StringVar(value="自动识别模式 2 / 3；模式 3 的 DLT_CTRL 优先读取 B / 配套 JSON。")
        self.calibration_text = tk.StringVar(value="前 3 秒静止零位标定：关闭")
        self.status = tk.StringVar(value="选择 USART6 日志，填写安装几何，然后点击“运行融合”。")
        self.rate_text = tk.StringVar()
        self.metrics = tk.StringVar()
        self.warnings = tk.StringVar()
        self.result: Optional[FusionResult] = None
        self.frame_times = []
        self.current_index = 0
        self.playing = False
        self.last_wall = 0.0
        self.target_mcu_ms = 0.0
        self.user_zoom = 1.0
        self.fit_center_e_m = 0.0
        self.fit_center_n_m = 0.0
        self.fit_span_m = 1.0
        self.pan_e_m = 0.0
        self.pan_n_m = 0.0
        self.drag_start = None
        self.visible_fused_points = []
        self.point_detail = tk.StringVar(value="滚轮缩放 · 左键拖动平移 · 悬停蓝点查看融合位置")
        self._slider_internal = False
        self._build()
        # KF-GINS hides this panel and keeps its own input configuration loader.
        if self.imu_input_frame.winfo_manager():
            self.log_path.trace_add("write", lambda *_: self._load_imu_profile())
            self._load_imu_profile()
        root.after(40, self._tick)
        if initial_log is not None:
            root.after(150, self._run)

    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)
        row = ttk.Frame(main)
        row.pack(fill="x")
        ttk.Label(row, text="日志文件").pack(side="left")
        ttk.Entry(row, textvariable=self.log_path).pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(row, text="选择", command=self._choose_log).pack(side="left")
        ttk.Button(row, text="运行融合", command=self._run).pack(side="left", padx=(8, 0))

        geometry = ttk.LabelFrame(main, text="安装几何与时间参数（可导入 / 保存 JSON）", padding=8)
        self.geometry_frame = geometry
        geometry.pack(fill="x", pady=(8, 0))
        for i, (key, title) in enumerate(LABELS):
            r, col = divmod(i, 3)
            cell = ttk.Frame(geometry)
            cell.grid(row=r, column=col, sticky="ew", padx=8, pady=3)
            ttk.Label(cell, text=title).pack(anchor="w")
            ttk.Entry(cell, textvariable=self.fields[key], width=20).pack(fill="x")
            geometry.columnconfigure(col, weight=1)
        geom_buttons = ttk.Frame(geometry)
        geom_buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(5, 0))
        ttk.Button(geom_buttons, text="导入安装配置", command=self._load_geometry).pack(side="left")
        ttk.Button(geom_buttons, text="保存安装配置", command=self._save_geometry).pack(side="left", padx=8)
        ttk.Button(geom_buttons, text="导出融合 CSV", command=self._export_csv).pack(side="left", padx=8)
        ttk.Label(geom_buttons, text="已填车辆默认安装；航向沿 IMU +X，可按实测修改。",
                  foreground="#92400e").pack(side="right")

        self.imu_input_frame = ttk.LabelFrame(main, text="IMU 输入（自动识别模式；以下设置仅作用于模式 3）", padding=6)
        self.imu_input_frame.pack(fill="x", pady=(7, 0))
        input_row = ttk.Frame(self.imu_input_frame)
        input_row.pack(fill="x")
        ttk.Label(input_row, text="DLT_CTRL（空白自动读取）").pack(side="left")
        ttk.Entry(input_row, textvariable=self.delta_ctrl, width=10).pack(side="left", padx=6)
        ttk.Checkbutton(input_row, text="模式3：应用板端启动标定（B/JSON）",
                        variable=self.apply_delta_startup).pack(side="left", padx=8)
        ttk.Label(self.imu_input_frame, textvariable=self.imu_input_text, wraplength=1080,
                  foreground="#1F4E79").pack(anchor="w")

        calibration_row = ttk.Frame(main)
        self.calibration_row = calibration_row
        calibration_row.pack(fill="x", pady=(7, 0))
        ttk.Checkbutton(calibration_row, text="启用前 3 秒静止 IMU 零位标定",
                        variable=self.calibrate_static_imu).pack(side="left")
        ttk.Label(calibration_row, text="需确认静止；勾选后点击“运行融合”重新计算",
                  foreground="#64748b").pack(side="left", padx=12)
        self.calibration_label = ttk.Label(main, textvariable=self.calibration_text,
                                           foreground="#0f766e", wraplength=1100)
        self.calibration_label.pack(anchor="w", pady=(3, 0))

        info = ttk.Frame(main)
        self.info_frame = info
        info.pack(fill="x", pady=(8, 0))
        ttk.Label(info, textvariable=self.status, font=("Microsoft YaHei UI", 10, "bold"),
                  foreground="#1f2937").pack(anchor="w")
        ttk.Label(info, textvariable=self.rate_text, foreground="#334155").pack(anchor="w", pady=(3, 0))
        ttk.Label(info, textvariable=self.warnings, foreground="#92400e",
                  wraplength=1100, justify="left").pack(anchor="w", pady=(3, 0))

        canvas_frame = ttk.LabelFrame(main, text="局部 EN 平面轨迹 / m（东向右，北向上）", padding=5)
        canvas_frame.pack(fill="both", expand=True, pady=(8, 0))
        self.canvas = tk.Canvas(canvas_frame, bg="#f8fafc", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self._draw())
        self.canvas.bind("<MouseWheel>", self._mouse_zoom)
        self.canvas.bind("<ButtonPress-1>", self._pan_start)
        self.canvas.bind("<B1-Motion>", self._pan_move)
        self.canvas.bind("<ButtonRelease-1>", self._pan_end)
        self.canvas.bind("<Motion>", self._hover_point)
        self.canvas.bind("<Leave>", lambda _: self.point_detail.set(
            "滚轮缩放 · 左键拖动平移 · 悬停蓝点查看融合位置"))

        legend = ttk.Frame(main)
        legend.pack(fill="x", pady=(6, 0))
        for text, color in (("● GNSS 主天线", "#94a3b8"),
                            ("● 杆臂修正观测", "#16a34a"),
                            ("━ ● 融合轨迹与位置点", "#2563eb"),
                            ("➤ 当前航向", "#ea580c")):
            label = ttk.Label(legend, text=text, foreground=color)
            label.pack(side="left", padx=(0, 16))
            if color == "#2563eb":
                self.fused_legend = label
        ttk.Button(legend, text="重置视图", command=self._reset_view).pack(side="right")
        ttk.Label(main, textvariable=self.point_detail, foreground="#334155").pack(anchor="w")

        controls = ttk.Frame(main)
        controls.pack(fill="x", pady=(6, 0))
        self.play_button = ttk.Button(controls, text="▶ 播放", command=self._toggle_play)
        self.play_button.pack(side="left")
        ttk.Label(controls, text="速度").pack(side="left", padx=(12, 3))
        ttk.Combobox(controls, textvariable=self.speed, values=("0.25×", "0.5×", "1×", "2×", "5×", "10×"),
                     width=7, state="readonly").pack(side="left")
        self.slider = ttk.Scale(controls, from_=0, to=1, command=self._scrub)
        self.slider.pack(side="left", fill="x", expand=True, padx=12)
        self.time_label = ttk.Label(controls, text="0.0 / 0.0 s", width=15)
        self.time_label.pack(side="right")
        ttk.Label(main, textvariable=self.metrics, font=("Consolas", 10)).pack(anchor="w", pady=(6, 0))
        self.model_note = ttk.Label(main, text="实验性二维松组合：支持模式 2 / 3 IMU，200 Hz 预测，UM982 10 Hz 位置 / 多普勒速度 / 双天线航向更新。"
                                    "未补偿横滚俯仰重力投影；PPS 的 GPS 整秒归属仍需核验，轨迹不能视为真值。",
                                    foreground="#6b7280", wraplength=1100)
        self.model_note.pack(anchor="w", pady=(3, 0))

    def _choose_log(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("USART6 日志", "*.txt"), ("全部文件", "*.*")])
        if path:
            self.log_path.set(path)

    def _geometry(self) -> Geometry:
        try:
            values = {key: float(self.fields[key].get()) for key, _ in LABELS}
            for key in ("gyro_heading_sign", "imu_y_right_sign"):
                if values[key] not in (-1.0, 1.0):
                    raise ValueError(f"{key} 必须为 +1 或 -1")
                values[key] = int(values[key])
            geometry = Geometry(**values)
            geometry.validate()
            return geometry
        except ValueError as exc:
            raise ValueError(f"安装参数无效：{exc}") from exc

    def _imu_options(self) -> ImuInputOptions:
        raw = self.delta_ctrl.get().strip()
        options = ImuInputOptions(int(raw, 0) if raw else None, self.apply_delta_startup.get())
        options.validate()
        return options

    def _load_imu_profile(self) -> None:
        path = Path(self.log_path.get())
        if not path.is_file():
            self.imu_input_text.set("自动识别模式 2 / 3；模式 3 的 DLT_CTRL 优先读取 B / 配套 JSON。")
            return
        try:
            mode = None
            with path.open("r", encoding="ascii", errors="replace") as source:
                for line in source:
                    if line.startswith("I,"):
                        fields = line.strip().split(",")
                        if len(fields) == 13:
                            mode = int(fields[2])
                            break
            if mode == 2:
                self.imu_input_text.set("检测到模式 2 原始角速度 / 加速度：保持原处理流程；本框的模式 3 设置不生效。")
                return
            if mode != 3:
                self.imu_input_text.set("尚未找到可识别的 IMU 帧，运行时将校验完整日志。")
                return
            metadata = load_capture(path, verify_log=True)
            startup = load_startup_calibration(path, metadata)
            ctrl = startup.delta_ctrl if startup else metadata["profile"]["delta_ctrl"] if metadata else None
            scale = f"记录 DLT_CTRL=0x{ctrl:04X}" if ctrl is not None else "缺少比例记录，需手填 DLT_CTRL"
            calibration = (f"板端标定 {startup.samples} 帧 / {startup.duration_us/1e6:.3f} s"
                           if startup else "无 B/JSON 标定，旧日志须明确取消板端标定选项")
            self.imu_input_text.set(f"检测到模式 3 角增量 / 速度增量；{scale}；{calibration}。")
        except (OSError, ValueError) as exc:
            self.imu_input_text.set(f"IMU 输入记录错误：{exc}")

    def _load_geometry(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("JSON 配置", "*.json")])
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            imu_options = ImuInputOptions(**data.pop("imu_input", {}))
            imu_options.validate()
            static = data.pop("calibrate_static_imu", False)
            if type(static) is not bool:
                raise ValueError("额外静止标定开关必须为布尔值")
            geometry = Geometry(**data)
            geometry.validate()
            for key, value in asdict(geometry).items():
                self.fields[key].set(str(value))
            self.delta_ctrl.set("" if imu_options.delta_ctrl is None else f"0x{imu_options.delta_ctrl:04X}")
            self.apply_delta_startup.set(imu_options.apply_startup_calibration)
            self.calibrate_static_imu.set(static)
        except (OSError, ValueError, TypeError) as exc:
            messagebox.showerror("导入失败", str(exc))

    def _save_geometry(self) -> None:
        try:
            geometry = self._geometry()
            imu_options = self._imu_options()
        except ValueError as exc:
            messagebox.showerror("配置错误", str(exc))
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="installation_geometry.json",
                                             filetypes=[("JSON 配置", "*.json")])
        if path:
            saved = dict(asdict(geometry), imu_input=asdict(imu_options),
                         calibrate_static_imu=self.calibrate_static_imu.get())
            Path(path).write_text(json.dumps(saved, ensure_ascii=False, indent=2) + "\n",
                                  encoding="utf-8")

    def _run(self) -> None:
        try:
            path = Path(self.log_path.get())
            if not path.is_file():
                raise ValueError("日志文件不存在")
            geometry = self._geometry()
            self.root.configure(cursor="watch")
            self.root.update_idletasks()
            result = self._compute(path, geometry)
        except (OSError, ValueError) as exc:
            messagebox.showerror("融合失败", str(exc))
            return
        finally:
            self.root.configure(cursor="")
        self.result = result
        self._fit_result()
        self.frame_times = [f.mcu_ms for f in result.frames]
        self.current_index = 0
        self.target_mcu_ms = self.frame_times[0]
        self.playing = False
        self.play_button.configure(text="▶ 播放")
        self.slider.configure(to=len(result.frames) - 1)
        self.slider.set(0)
        self.user_zoom = 1.0
        self.pan_e_m = 0.0
        self.pan_n_m = 0.0
        self.drag_start = None
        span = (self.frame_times[-1] - self.frame_times[0]) / 1000
        corr = (f"；Z角速度/航向相关 {result.gyro_heading_correlation:+.2f}"
                if result.gyro_heading_correlation is not None else "")
        timing = (f"PPS/DRDY 硬件时基 ({result.pps_count} PPS)"
                  if result.hardware_sync else
                  f"GPS/MCU 到达拟合残差 {result.gps_arrival_fit_rms_ms:.2f} ms")
        self.status.set(f"已融合 {result.imu_count} 帧 IMU、{result.position_count} 条位置、"
                        f"{result.heading_count} 条航向；回放 {span:.1f} 秒。"
                        f" {timing}；"
                        f"坏记录 {result.rejected_lines}{corr}。")
        fusion_dt = self.frame_times[-1] - self.frame_times[0]
        gnss_dt = result.gnss[-1].mcu_ms - result.gnss[0].mcu_ms
        fusion_hz = ((len(result.frames) - 1) * 1000 / fusion_dt) if fusion_dt > 0 else 0.0
        gnss_hz = ((len(result.gnss) - 1) * 1000 / gnss_dt) if gnss_dt > 0 else 0.0
        self.rate_text.set(f"融合坐标 {fusion_hz:.1f} Hz（每个有效 IMU 帧一条）；"
                           f"GNSS 位置修正 {gnss_hz:.1f} Hz；画面蓝点会按密度抽样显示。")
        calibration = result.static_calibration
        if calibration is None:
            self.calibration_text.set("前 3 秒静止零位标定：关闭")
        elif calibration.applied:
            self.calibration_text.set(
                f"前 3 秒静止零位标定：已应用 {calibration.sample_count} 帧；"
                f"角速度 XYZ=({calibration.gyro_x_dps:+.4f}, "
                f"{calibration.gyro_y_dps:+.4f}, {calibration.gyro_z_dps:+.4f}) °/s；"
                f"X 加速度 {calibration.accel_x_mps2:+.4f} m/s²，"
                f"Y 加速度 {calibration.accel_y_mps2:+.4f} m/s²")
        else:
            self.calibration_text.set(
                f"前 3 秒静止零位标定：未应用（{calibration.reason}）；"
                "本次未额外扣除此段均值")
        if getattr(result, "imu_input_mode", 2) == 3:
            startup = result.board_startup_calibration
            if startup:
                mean = " / ".join(f"{degrees(v):+.6f}" for v in startup.stationary_rate)
                self.imu_input_text.set(f"本次：模式 3，DLT_CTRL=0x{result.delta_ctrl:04X}；"
                                        f"已扣板端 XYZ 静止均值 {mean} °/s；原日志保持不变。")
                if calibration:
                    self.calibration_text.set(self.calibration_text.get().replace(
                        "前 3 秒静止零位标定", "前 3 秒额外标定（板端补偿后）"))
            else:
                self.imu_input_text.set(f"本次：模式 3，DLT_CTRL=0x{result.delta_ctrl:04X}；未应用板端启动标定。")
        notes = list(result.warnings)
        if self._zero_geometry(geometry):
            notes.insert(0, "当前为零杆臂/零偏角演示；填写实测安装值后重新运行")
        self.warnings.set("；".join(notes))
        self._draw()

    def _compute(self, path: Path, geometry: Geometry) -> FusionResult:
        return run_fusion(path, geometry,
                          calibrate_static_imu=self.calibrate_static_imu.get(), imu_options=self._imu_options())

    def _frame_position(self, frame) -> tuple[float, float]:
        """Display hook; the 2-D tool continues to use its original IMU coordinates."""
        return frame.east_m, frame.north_m

    def _fit_result(self) -> None:
        if self.result is None:
            return
        points = [(g.antenna_e_m, g.antenna_n_m) for g in self.result.gnss]
        points += [self._frame_position(f) for f in self.result.frames]
        xs, ys = zip(*points)
        self.fit_center_e_m = (min(xs) + max(xs)) / 2
        self.fit_center_n_m = (min(ys) + max(ys)) / 2
        self.fit_span_m = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) * 1.18

    def _zero_geometry(self, geometry: Geometry) -> bool:
        return geometry.lever_x_m == geometry.lever_y_m == geometry.heading_offset_deg == 0

    def _export_csv(self) -> None:
        if self.result is None:
            messagebox.showinfo("导出", "请先运行融合。")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="fusion_2d.csv",
                                             filetypes=[("CSV 表格", "*.csv")])
        if not path:
            return
        first_t = self.result.frames[0].mcu_ms
        with Path(path).open("w", encoding="utf-8", newline="") as file:
            file.write("elapsed_s,mcu_ms,east_m,north_m,ve_mps,vn_mps,heading_deg,gyro_bias_dps\n")
            for f in self.result.frames:
                file.write(f"{(f.mcu_ms-first_t)/1000:.6f},{f.mcu_ms:.3f},{f.east_m:.6f},"
                           f"{f.north_m:.6f},{f.ve_mps:.6f},{f.vn_mps:.6f},"
                           f"{f.heading_deg:.6f},{f.gyro_bias_dps:.6f}\n")
        self.status.set(f"已导出 {len(self.result.frames)} 条融合结果：{path}")

    def _toggle_play(self) -> None:
        if self.result is None:
            return
        if not self.playing and self.current_index >= len(self.result.frames) - 1:
            self.current_index = 0
            self.target_mcu_ms = self.frame_times[0]
        self.playing = not self.playing
        self.last_wall = time.monotonic()
        self.play_button.configure(text="❚❚ 暂停" if self.playing else "▶ 播放")

    def _scrub(self, value: str) -> None:
        if self.result is None or self._slider_internal:
            return
        self.playing = False
        self.play_button.configure(text="▶ 播放")
        self.current_index = max(0, min(len(self.result.frames) - 1, round(float(value))))
        self.target_mcu_ms = self.frame_times[self.current_index]
        self._draw()

    def _tick(self) -> None:
        if self.playing and self.result is not None:
            now = time.monotonic()
            factor = float(self.speed.get().rstrip("×"))
            self.target_mcu_ms += (now - self.last_wall) * 1000 * factor
            self.last_wall = now
            index = bisect_right(self.frame_times, self.target_mcu_ms) - 1
            self.current_index = min(max(index, 0), len(self.frame_times) - 1)
            self._slider_internal = True
            self.slider.set(self.current_index)
            self._slider_internal = False
            self._draw()
            if self.current_index == len(self.frame_times) - 1:
                self.playing = False
                self.play_button.configure(text="▶ 播放")
        self.root.after(40, self._tick)

    def _reset_view(self) -> None:
        self.user_zoom = 1.0
        self.pan_e_m = 0.0
        self.pan_n_m = 0.0
        self._draw()

    def _mouse_zoom(self, event: tk.Event) -> None:
        if self.result is None or not event.delta:
            return
        old_scale = self._view_scale()
        if old_scale <= 0:
            return
        new_zoom = min(100, max(0.3, self.user_zoom * (1.2 if event.delta > 0 else 1 / 1.2)))
        if new_zoom == self.user_zoom:
            return
        new_scale = old_scale * new_zoom / self.user_zoom
        dx = event.x - self.canvas.winfo_width() / 2
        dy = event.y - self.canvas.winfo_height() / 2
        self.pan_e_m += dx * (1 / old_scale - 1 / new_scale)
        self.pan_n_m -= dy * (1 / old_scale - 1 / new_scale)
        self.user_zoom = new_zoom
        self._draw()

    def _pan_start(self, event: tk.Event) -> None:
        if self.result is not None:
            self.drag_start = (event.x, event.y, self.pan_e_m, self.pan_n_m)
            self.canvas.configure(cursor="fleur")

    def _pan_move(self, event: tk.Event) -> None:
        if self.drag_start is None:
            return
        scale = self._view_scale()
        if scale <= 0:
            return
        x, y, pan_e, pan_n = self.drag_start
        self.pan_e_m = pan_e - (event.x - x) / scale
        self.pan_n_m = pan_n + (event.y - y) / scale
        self._draw()

    def _pan_end(self, _event: tk.Event) -> None:
        self.drag_start = None
        self.canvas.configure(cursor="")

    def _hover_point(self, event: tk.Event) -> None:
        if self.drag_start is not None or self.result is None:
            return
        near = min(self.visible_fused_points,
                   key=lambda p: (p[0] - event.x)**2 + (p[1] - event.y)**2,
                   default=None)
        if near is None or (near[0] - event.x)**2 + (near[1] - event.y)**2 > 100:
            self.point_detail.set("滚轮缩放 · 左键拖动平移 · 悬停蓝点查看融合位置")
            return
        frame = self.result.frames[near[2]]
        east, north = self._frame_position(frame)
        elapsed = (frame.mcu_ms - self.result.frames[0].mcu_ms) / 1000
        self.point_detail.set(f"融合点 #{near[2] + 1}  t={elapsed:.3f} s  "
                              f"E={east:+.3f} m  N={north:+.3f} m  "
                              f"速度={hypot(frame.ve_mps, frame.vn_mps):.3f} m/s")

    def _view_scale(self) -> float:
        if self.result is None:
            return 0.0
        width = max(self.canvas.winfo_width(), 200)
        height = max(self.canvas.winfo_height(), 200)
        return min(width - 105, height - 90) / self.fit_span_m * self.user_zoom

    @staticmethod
    def _nice_step(span: float) -> float:
        raw = max(span / 7, 1e-5)
        decade = 10**floor(log10(raw))
        return next((n * decade for n in (1, 2, 5, 10) if n * decade >= raw), 10 * decade)

    def _draw(self) -> None:
        if self.result is None:
            return
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 200), max(canvas.winfo_height(), 200)
        usable_w, usable_h = width - 105, height - 90
        if usable_w <= 0 or usable_h <= 0:
            return
        fused = self.result.frames
        gnss = self.result.gnss
        cx = self.fit_center_e_m + self.pan_e_m
        cy = self.fit_center_n_m + self.pan_n_m
        scale = min(usable_w, usable_h) / self.fit_span_m * self.user_zoom
        center_x, center_y = width / 2, height / 2

        def xy(e: float, n: float) -> tuple:
            return center_x + (e - cx) * scale, center_y - (n - cy) * scale

        visible = max(usable_w, usable_h) / scale
        step = self._nice_step(visible)
        xmin, xmax = cx - width / (2 * scale), cx + width / (2 * scale)
        ymin, ymax = cy - height / (2 * scale), cy + height / (2 * scale)
        for k in range(ceil(xmin / step), floor(xmax / step) + 1):
            e = k * step
            x, _ = xy(e, 0)
            canvas.create_line(x, 0, x, height, fill="#e2e8f0")
            canvas.create_text(x + 3, height - 8, text=f"{e:g}", anchor="sw", fill="#64748b")
        for k in range(ceil(ymin / step), floor(ymax / step) + 1):
            n = k * step
            _, y = xy(0, n)
            canvas.create_line(0, y, width, y, fill="#e2e8f0")
            canvas.create_text(7, y - 3, text=f"{n:g}", anchor="sw", fill="#64748b")
        canvas.create_text(width - 12, height - 26, text="E / m →", anchor="se", fill="#475569")
        canvas.create_text(10, 10, text="N / m ↑", anchor="nw", fill="#475569")
        ox, oy = xy(0, 0)
        canvas.create_line(ox - 6, oy, ox + 6, oy, fill="#475569")
        canvas.create_line(ox, oy - 6, ox, oy + 6, fill="#475569")
        end_t = fused[self.current_index].mcu_ms
        for g in gnss:
            if g.mcu_ms > end_t:
                break
            gx, gy = xy(g.antenna_e_m, g.antenna_n_m)
            if -5 <= gx <= width + 5 and -5 <= gy <= height + 5:
                canvas.create_oval(gx - 2, gy - 2, gx + 2, gy + 2, fill="#94a3b8", outline="")
            if abs(g.imu_e_m - g.antenna_e_m) + abs(g.imu_n_m - g.antenna_n_m) > 1e-4:
                qx, qy = xy(g.imu_e_m, g.imu_n_m)
                if -5 <= qx <= width + 5 and -5 <= qy <= height + 5:
                    canvas.create_oval(qx - 2, qy - 2, qx + 2, qy + 2, fill="#16a34a", outline="")
        visible_indices = []
        for i, frame in enumerate(fused[:self.current_index + 1]):
            fx, fy = xy(*self._frame_position(frame))
            if -5 <= fx <= width + 5 and -5 <= fy <= height + 5:
                visible_indices.append((i, fx, fy))
        # Keep line detail in the zoomed region without creating thousands of canvas objects.
        line_stride = max(1, ceil((self.current_index + 1) / 4000))
        marker_stride = max(1, ceil(len(visible_indices) / 700))
        line_indices = set(range(0, self.current_index + 1, line_stride))
        line_indices.update(i for i, _, _ in visible_indices[::marker_stride])
        line_indices.add(self.current_index)
        path = []
        for i in sorted(line_indices):
            path.extend(xy(*self._frame_position(fused[i])))
        if len(path) >= 4:
            canvas.create_line(*path, fill="#2563eb", width=2.5, smooth=False)
        self.visible_fused_points = []
        for i, fx, fy in visible_indices[::marker_stride]:
            canvas.create_oval(fx - 2.5, fy - 2.5, fx + 2.5, fy + 2.5,
                               fill="#2563eb", outline="#ffffff", width=0.7)
            self.visible_fused_points.append((fx, fy, i))
        frame = fused[self.current_index]
        east, north = self._frame_position(frame)
        px, py = xy(east, north)
        canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill="#ea580c", outline="white", width=2)
        angle = radians(frame.heading_deg)
        canvas.create_line(px, py, px + 25 * sin(angle), py - 25 * cos(angle),
                           fill="#ea580c", width=3, arrow=tk.LAST)
        elapsed = (frame.mcu_ms - fused[0].mcu_ms) / 1000
        total = (fused[-1].mcu_ms - fused[0].mcu_ms) / 1000
        self.time_label.configure(text=f"{elapsed:.1f} / {total:.1f} s")
        self.metrics.set(f"当前 E={east:+.3f} m   N={north:+.3f} m   "
                         f"速度={hypot(frame.ve_mps, frame.vn_mps):.3f} m/s   "
                         f"航向={frame.heading_deg:.2f}°   "
                         f"原点=({self.result.origin_lat_deg:.8f}°, {self.result.origin_lon_deg:.8f}°)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, help="可选的 USART6 日志路径")
    args = parser.parse_args()
    root = tk.Tk()
    ReplayApp(root, args.log)
    root.mainloop()


if __name__ == "__main__":
    main()
