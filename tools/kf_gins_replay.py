"""TerraMind KF-GINS offline replay for mode-3 incremental IMU logs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from fusion2d import FusionResult, Geometry
from fusion_replay import ReplayApp
from kf_gins_bridge import KfOptions, run_kf_gins
from capture_profile import load_capture


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
        defaults = KfOptions()
        self.kf_fields = {}
        self.kf_defaults = defaults
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
            metadata = load_capture(Path(self.log_path.get()))
            if metadata is None:
                self.status.set("无配套采集配置，请手动核对 DLT_CTRL；运行时仍检查实际日志模式。")
                return
            profile = metadata["profile"]
            self.kf_fields["delta_ctrl"].set(f"0x{profile['delta_ctrl']:04X}")
            state = "人工已核对" if profile["delta_ctrl_confirmed"] else "未核对，须检查实际寄存器"
            self.status.set(f"已读取采集配置：期望模式 {profile['expected_mode']}，DLT_CTRL {state}。")
        except (OSError, ValueError) as exc:
            self.status.set(f"配套采集配置错误：{exc}")

    def _build(self) -> None:
        super()._build()
        self.geometry_frame.pack_forget()
        self.calibration_row.pack_forget()
        self.calibration_label.pack_forget()
        self.model_note.configure(text="KF-GINS 三维 21 状态松组合：角/速度增量预测，GNSS 位置更新。"
                                       "双天线航向仅用于初始航向；结果精度取决于实测安装、初始姿态、IMU 标定和时间同步。")
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
        buttons = ttk.Frame(box)
        buttons.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(5, 0))
        ttk.Button(buttons, text="导入 KF 配置", command=self._load_kf_options).pack(side="left")
        ttk.Button(buttons, text="保存 KF 配置", command=self._save_kf_options).pack(side="left", padx=8)
        ttk.Button(buttons, text="导出三维结果 CSV", command=self._export_csv).pack(side="left", padx=8)
        ttk.Button(buttons, text="选择 KF 噪声配置 YAML", command=self._choose_template).pack(side="left", padx=8)
        self.config_path = tk.StringVar(value="")
        ttk.Label(box, textvariable=self.config_path, foreground="#475569").grid(
            row=4, column=0, columnspan=4, sticky="w")

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
        if f.height_m is not None:
            self.metrics.set(self.metrics.get() +
                             f"   高程={f.height_m:.3f} m   横滚={f.roll_deg:.2f}°   俯仰={f.pitch_deg:.2f}°")

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
