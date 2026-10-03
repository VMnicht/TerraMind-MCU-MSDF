"""TerraMind KF-GINS offline replay for mode-3 incremental IMU logs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
from math import degrees
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from fusion2d import FusionResult, Geometry
from fusion_replay import ReplayApp
from kf_gins_bridge import KfOptions, VEHICLE_CONFIG, run_kf_gins
from capture_profile import load_capture
from startup_calibration import load_startup_calibration
from kf_gins_diagnostics import HeadingComparisonWindow, write_diagnostics_csv


FIELDS = (
    ("delta_ctrl", "G365 DLT_CTRL (十六进制)"),
    ("axis_forward", "车体前轴对应 IMU 轴"),
    ("axis_right", "车体右轴对应 IMU 轴"),
    ("axis_down", "车体下轴对应 IMU 轴"),
    ("lever_forward_m", "主天线相对 IMU 前向 / m"),
    ("lever_right_m", "主天线相对 IMU 右向 / m"),
    ("lever_down_m", "主天线相对 IMU 下向 / m"),
    ("heading_offset_deg", "双天线航向 → IMU 前向偏角 / °"),
    ("initial_roll_deg", "初始横滚 / °"),
    ("initial_pitch_deg", "初始俯仰 / °"),
    ("initial_heading_deg", "初始航向 / °（空白自动取双天线）"),
    ("gnss_delay_ms", "无 PPS 时 GNSS 历元提前量 / ms"),
)


class KfReplayApp(ReplayApp):
    def __init__(self, root: tk.Tk, initial_log: Path | None = None) -> None:
        # User-confirmed vehicle installation; capture metadata can still supply
        # the actual DLT_CTRL for historical logs, and imported settings override.
        defaults = KfOptions(
            delta_ctrl=0x0048,
            axis_forward="+y", axis_right="-x", axis_down="+z",
            lever_forward_m=0.0, lever_right_m=-0.15, lever_down_m=0.0,
            heading_offset_deg=-90.0,
            initial_roll_deg=0.0, initial_pitch_deg=0.0,
            initial_heading_deg=None, gnss_delay_ms=0.0,
            config_path=str(VEHICLE_CONFIG), heading_aiding=True, velocity_aiding=True,
        )
        self.kf_fields = {}
        self.kf_defaults = defaults
        self.display_reference = tk.StringVar(master=root, value="主天线（与 GNSS 同点）")
        self.heading_comparison_text = tk.StringVar(master=root, value="航向对比：运行融合后显示。")
        self.heading_window = None
        super().__init__(root, initial_log)
        root.title("TerraMind KF-GINS 三维组合导航离线回放")
        root.geometry("1200x950")
        self.status.set("选择模式 3 增量 IMU 的 USART6 日志，核对安装参数，点击“运行融合”。")
        self.log_path.trace_add("write", lambda *_: self._load_capture_profile())
        self._load_capture_profile()

    def _load_capture_profile(self) -> None:
        try:
            if not self.log_path.get().strip():
                return
            path = Path(self.log_path.get())
            metadata = load_capture(path, verify_log=True)
            if metadata:
                profile = metadata["profile"]
                self.kf_fields["delta_ctrl"].set(f"0x{profile['delta_ctrl']:04X}")
                state = "人工已核对" if profile["delta_ctrl_confirmed"] else "未核对，须检查实际寄存器"
                self.status.set(f"已读取采集配置：期望模式 {profile['expected_mode']}，DLT_CTRL {state}。")
            else:
                self.status.set("无配套采集配置，读取日志 B 中的启动标定。")
            startup = load_startup_calibration(path, metadata)
            if startup:
                if metadata is None:
                    self.kf_fields["delta_ctrl"].set(f"0x{startup.delta_ctrl:04X}")
                mean = " / ".join(f"{degrees(v):+.6f}" for v in startup.stationary_rate)
                self.startup_text.set(f"已读标定：{startup.samples} 帧 / {startup.duration_us/1e6:.3f} s，"
                                      f"DLT_CTRL=0x{startup.delta_ctrl:04X}；XYZ 静止均值 {mean} °/s。")
            else:
                self.startup_text.set("未找到启动标定；启用标定时不能运行，旧日志可手动取消勾选以分析原始数据。")
        except (OSError, ValueError) as exc:
            self.startup_text.set(f"标定/采集配置错误：{exc}")
            self.status.set(f"配套采集配置错误：{exc}")

    def _build(self) -> None:
        super()._build()
        self.geometry_frame.pack_forget()
        self.imu_input_frame.pack_forget()
        self.calibration_row.pack_forget()
        self.calibration_label.pack_forget()
        self.model_note.configure(text="KF-GINS 完整三维 21 状态：默认使用 G365 车载调优参数，持续位置 / 双天线航向 / 多普勒速度更新。"
                                       "可切换原版参数对照，或导入自定义 YAML。")
        box = ttk.LabelFrame(self.info_frame.master, text="KF-GINS 输入参数（前 / 右 / 下右手系）", padding=8)
        box.pack(fill="x", pady=(8, 0), before=self.info_frame)
        defaults = asdict(self.kf_defaults)
        for i, (key, label) in enumerate(FIELDS):
            row, column = divmod(i, 4)
            cell = ttk.Frame(box)
            cell.grid(row=row, column=column, sticky="ew", padx=5, pady=3)
            ttk.Label(cell, text=label).pack(anchor="w")
            value = defaults[key]
            if key == "delta_ctrl":
                value = f"0x{value:04X}"
            elif value is None:
                value = ""
            var = tk.StringVar(value=str(value))
            self.kf_fields[key] = var
            ttk.Entry(cell, textvariable=var).pack(fill="x")
            box.columnconfigure(column, weight=1)
        self.apply_startup = tk.BooleanVar(value=self.kf_defaults.apply_startup_calibration)
        ttk.Checkbutton(box, text="应用板端启动标定（扣除三轴陀螺静止均值，包含地球自转；不补偿加速度）",
                        variable=self.apply_startup).grid(row=3, column=0, columnspan=4, sticky="w")
        self.startup_text = tk.StringVar(value="选择日志后读取 B / 配套 JSON 中的启动标定。")
        ttk.Label(box, textvariable=self.startup_text, wraplength=1080, foreground="#1F4E79").grid(
            row=4, column=0, columnspan=4, sticky="w")
        buttons = ttk.Frame(box)
        buttons.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(5, 0))
        ttk.Button(buttons, text="导入 KF 配置", command=self._load_kf_options).pack(side="left")
        ttk.Button(buttons, text="保存 KF 配置", command=self._save_kf_options).pack(side="left", padx=8)
        ttk.Button(buttons, text="导出三维结果 CSV（IMU）", command=self._export_csv).pack(side="left", padx=8)
        ttk.Button(buttons, text="选择 KF 噪声配置 YAML", command=self._choose_template).pack(side="left", padx=8)
        self.config_path = tk.StringVar(value=self.kf_defaults.config_path)
        ttk.Label(box, textvariable=self.config_path, foreground="#475569").grid(
            row=6, column=0, columnspan=4, sticky="w")
        display = ttk.Frame(box)
        display.grid(row=7, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Label(display, text="融合轨迹参考点").pack(side="left")
        reference = ttk.Combobox(display, textvariable=self.display_reference, state="readonly",
                                 values=("主天线（与 GNSS 同点）", "IMU（原始解算位置）"), width=25)
        reference.pack(side="left", padx=6)
        reference.bind("<<ComboboxSelected>>", lambda _: self._change_reference())
        ttk.Button(display, text="航向 / 残差曲线", command=self._show_heading_comparison).pack(side="left", padx=6)
        ttk.Button(display, text="导出诊断 CSV", command=self._export_diagnostics).pack(side="left", padx=6)
        ttk.Label(box, textvariable=self.heading_comparison_text, foreground="#7c3aed",
                  wraplength=1080).grid(row=8, column=0, columnspan=4, sticky="w", pady=(4, 0))
        aiding = ttk.Frame(box)
        aiding.grid(row=9, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        self.heading_aiding = tk.BooleanVar(value=self.kf_defaults.heading_aiding)
        self.velocity_aiding = tk.BooleanVar(value=self.kf_defaults.velocity_aiding)
        ttk.Checkbutton(aiding, text="持续双天线航向更新（离线实验）",
                        variable=self.heading_aiding).pack(side="left")
        ttk.Checkbutton(aiding, text="GNSS 多普勒速度更新（含杆臂）",
                        variable=self.velocity_aiding).pack(side="left", padx=8)
        ttk.Button(aiding, text="车载调优参数", command=lambda: self._set_tuning(True)).pack(side="left", padx=4)
        ttk.Button(aiding, text="原版对照参数", command=lambda: self._set_tuning(False)).pack(side="left", padx=4)
        self._change_reference()

    def _set_tuning(self, vehicle: bool) -> None:
        # Only noise / initial uncertainty and aiding switches change here.
        # Preserve the selected log, sensor calibration and measured installation.
        self.config_path.set(str(VEHICLE_CONFIG) if vehicle else "")
        self.heading_aiding.set(vehicle)
        self.velocity_aiding.set(vehicle)
        name = "G365 车载调优（位置 + 航向 + 速度）" if vehicle else "上游默认参数（仅位置）"
        self.status.set(f"已选择{name}；安装参数保持当前值，请重新运行融合。")

    def _options(self) -> KfOptions:
        try:
            values = {}
            for key, _ in FIELDS:
                raw = self.kf_fields[key].get().strip()
                if key == "delta_ctrl":
                    values[key] = int(raw, 0)
                elif key.startswith("axis_"):
                    values[key] = raw.lower()
                elif key == "initial_heading_deg":
                    values[key] = float(raw) if raw else None
                else:
                    values[key] = float(raw)
            values["config_path"] = self.config_path.get()
            values["apply_startup_calibration"] = self.apply_startup.get()
            values["heading_aiding"] = self.heading_aiding.get()
            values["velocity_aiding"] = self.velocity_aiding.get()
            options = KfOptions(**values)
            options.validate()
            return options
        except (ValueError, TypeError) as exc:
            raise ValueError(f"KF-GINS 参数无效：{exc}") from exc

    def _geometry(self) -> Geometry:
        options = self._options()
        self.current_options = options
        return Geometry(options.lever_forward_m, options.lever_right_m,
                        options.heading_offset_deg, 1, 1, options.gnss_delay_ms)

    def _compute(self, path: Path, geometry: Geometry) -> FusionResult:
        return run_kf_gins(path, self.current_options)

    def _zero_geometry(self, geometry: Geometry) -> bool:
        options = self.current_options
        return options.lever_forward_m == options.lever_right_m == options.lever_down_m == options.heading_offset_deg == 0

    def _run(self) -> None:
        previous = self.result
        super()._run()
        if self.result is not None and self.result is not previous:
            self.status.set("KF-GINS 三维解算完成。" + self.status.get())
            self.calibration_text.set("")
            self._draw()

    def _draw(self) -> None:
        super()._draw()
        if self.result is None:
            return
        f = self.result.frames[self.current_index]
        label = "主天线估计位置" if self._antenna_view() else "IMU 位置"
        self.metrics.set(label + "  " + self.metrics.get().replace("速度=", "IMU速度="))
        if f.height_m is not None:
            self.metrics.set(self.metrics.get() +
                             f"\nIMU高程={f.height_m:.3f} m   横滚={f.roll_deg:.2f}°   俯仰={f.pitch_deg:.2f}°")
        if f.heading_residual_deg is None:
            self.heading_comparison_text.set(f"融合航向 {f.heading_deg:.2f}°；当前无有效双天线航向对照（缺测段不外推）。")
        else:
            use = ("本次已启用航向更新；此处为同源观测残差。" if getattr(self.result, "heading_aiding", False)
                   else "本次航向仅用于初始化和诊断，未持续更新。")
            self.heading_comparison_text.set(
                f"融合航向 {f.heading_deg:.2f}°；双天线车头航向 {f.gnss_vehicle_heading_deg:.2f}°；"
                f"残差（融合−双天线）{f.heading_residual_deg:+.2f}°；参考标准差 {f.gnss_heading_std_deg:.2f}°。{use}")
        if self.heading_window and self.heading_window.exists():
            self.heading_window.redraw()

    def _antenna_view(self) -> bool:
        return self.display_reference.get().startswith("主天线")

    def _frame_position(self, frame) -> tuple[float, float]:
        if self._antenna_view() and getattr(frame, "antenna_east_m", None) is not None:
            return frame.antenna_east_m, frame.antenna_north_m
        return super()._frame_position(frame)

    def _change_reference(self) -> None:
        label = "融合主天线位置" if self._antenna_view() else "融合 IMU 位置"
        self.fused_legend.configure(text="━ ● " + label)
        self.point_detail.set("当前蓝线：" + label + "；滚轮缩放、拖动平移、悬停查看坐标。")
        self._fit_result()
        self._reset_view()

    def _hover_point(self, event) -> None:
        super()._hover_point(event)
        if self.point_detail.get().startswith("融合点"):
            label = "主天线估计" if self._antenna_view() else "IMU"
            self.point_detail.set(label + " " + self.point_detail.get().replace("速度=", "IMU速度="))

    def _show_heading_comparison(self) -> None:
        if self.result is None:
            messagebox.showinfo("航向对比", "请先运行 KF-GINS。")
            return
        if self.heading_window and self.heading_window.exists():
            self.heading_window.window.lift()
            self.heading_window.redraw()
        else:
            self.heading_window = HeadingComparisonWindow(self)

    def _export_diagnostics(self) -> None:
        if self.result is None:
            messagebox.showinfo("导出", "请先运行 KF-GINS。")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="kf_gins_diagnostics.csv",
                                             filetypes=[("CSV 表格", "*.csv")])
        if path:
            write_diagnostics_csv(self.result, Path(path))
            self.status.set(f"已导出诊断 CSV（同时保留 IMU / 主天线位置和航向残差）：{path}")

    def _choose_template(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("YAML", "*.yaml *.yml"), ("全部文件", "*.*")])
        if path:
            self.config_path.set(path)

    def _load_kf_options(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("JSON 配置", "*.json")])
        if not path:
            return
        try:
            options = KfOptions(**json.loads(Path(path).read_text(encoding="utf-8")))
            options.validate()
            for key, _ in FIELDS:
                value = getattr(options, key)
                self.kf_fields[key].set("" if value is None else f"0x{value:04X}" if key == "delta_ctrl" else str(value))
            self.config_path.set(options.config_path)
            self.apply_startup.set(options.apply_startup_calibration)
            self.heading_aiding.set(options.heading_aiding)
            self.velocity_aiding.set(options.velocity_aiding)
        except (OSError, ValueError, TypeError) as exc:
            messagebox.showerror("导入失败", str(exc))

    def _save_kf_options(self) -> None:
        try:
            options = self._options()
        except ValueError as exc:
            messagebox.showerror("配置错误", str(exc))
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="kf_gins_installation.json",
                                             filetypes=[("JSON 配置", "*.json")])
        if path:
            Path(path).write_text(json.dumps(asdict(options), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _export_csv(self) -> None:
        if self.result is None:
            messagebox.showinfo("导出", "请先运行 KF-GINS。")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="kf_gins_3d.csv",
                                             filetypes=[("CSV 表格", "*.csv")])
        if not path:
            return
        first = self.result.frames[0].mcu_ms
        with Path(path).open("w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(("elapsed_s", "gps_week", "gps_tow_s", "lat_deg", "lon_deg", "height_m",
                             "east_m", "north_m", "vn_mps", "ve_mps", "vd_mps",
                             "roll_deg", "pitch_deg", "heading_deg"))
            for f in self.result.frames:
                writer.writerow((f"{(f.mcu_ms - first) / 1000:.6f}", f.gps_week,
                                 f"{f.mcu_ms / 1000:.9f}",
                                 f.lat_deg, f.lon_deg, f.height_m, f.east_m, f.north_m,
                                 f.vn_mps, f.ve_mps, f.vd_mps, f.roll_deg, f.pitch_deg, f.heading_deg))
        self.status.set(f"已导出 {len(self.result.frames)} 条三维导航结果：{path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, help="USART6 模式 3 原始日志")
    args = parser.parse_args()
    root = tk.Tk()
    KfReplayApp(root, args.log)
    root.mainloop()


if __name__ == "__main__":
    main()
