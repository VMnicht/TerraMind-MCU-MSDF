# G365PDF1 UART4 burst 解包

UART4 使用 230400 baud。当前设备使用上位机的配置 2，因此 STM32 解包器默认选择 `APP_IMU_MODE_RAW32`。将传感器切到配置 3 后，再调用 `App_ImuSetMode(APP_IMU_MODE_DELTA32)`；该调用只切换 STM32 的解包方式，不写传感器寄存器。

| 上位机模式 | BURST_CTRL1 | BURST_CTRL2 | 上位机 ATTI_CTRL | 字节 5–16 | 字节 17–28 |
| --- | --- | --- | --- | --- | --- |
| 配置 2：原始 IMU 32 位 | `F007` | `3000` | `00` | X/Y/Z 角速度 | X/Y/Z 加速度 |
| 配置 3：增量导航 32 位 | `CC07` | `0C00` | `02` | X/Y/Z 增量角 | X/Y/Z 增量速度 |

表中寄存器值是现有 G366 上位机的配置。G365PDF1 的增量输出使能位位于 `ATTI_CTRL` 高字节 bit1；上位机的配置 3 寄存器写入需按 G365PDF1 核对，STM32 解包器只处理已收到的帧。

两种模式均为 36 字节。字节 0 为 `80`；1–2 为 FLAG；3–4 为有符号 16 位温度；5–28 为六个有符号 32 位值；29–30 为 GPIO；31–32 为 COUNT；33–34 为 CHECKSUM；35 为 `0D`。多字节字段均为大端。CHECKSUM 为字节 1–32 的十六个大端 16 位字之和，取低 16 位。

两种模式的帧长和校验方式完全相同，无法从收到的字节自动判断模式。模式切换时解包器丢弃已缓存字节、未完成帧、上一帧结果并清零协议统计。应在传感器完成对应配置切换、输出稳定后调用 `App_ImuSetMode`；切换后 `App_ImuGetLatest` 要等首帧新数据才返回 1。

`AppImuSample` 保留原始寄存器整数。配置 2 填充 `gyro_raw`、`accel_raw`；配置 3 填充 `delta_angle_raw`、`delta_velocity_raw`；另一组字段为零。使用前检查 `sample.mode`。当前 200 Hz 抓包的 COUNT 每帧交替增加 312/313；增加约 625 表示间隔了约两个采样周期。COUNT 仅用于跳帧统计，并非帧解包条件。

`G365Imu::Sample` 同时提供独立的 32 位浮点成员：`temperature_c`、`gyro_x_dps`/`y`/`z`、`accel_x_g`/`y`/`z`、`delta_angle_x_deg`/`y`/`z`、`delta_velocity_x_mps`/`y`/`z`。Keil Watch 可直接输入 `g_g365_debug_sample.gyro_x_dps`，其他轴改字段名即可；该全局样本只在校验通过后更新。当前模式没有发送的量保持 0。

浮点数按 G365PDF1 的系数换算：温度 °C = `25 + (temperature_raw - 2634) × (-0.0037918)`；配置 2 的角速度 °/s = `gyro_raw / (66 × 65536)`，加速度 g = `accel_raw / (2.5 × 1000 × 65536)`。G365PDF1 为固定 ±10 g 量程，不能按 G366 的 `4/2 LSB/mg` 换算。配置 3 的增量角 ° = `delta_angle_raw × ((1/66/2000) × 2^角度代码) / 65536`，增量速度 m/s = `delta_velocity_raw × ((0.4/1000 × 9.80665/2000) × 2^速度代码) / 65536`。

G365PDF1 的两个增量代码位于 `DLT_CTRL` 低字节；STM32 不主动读取该寄存器，目前以 `0x00CC` 作为默认**假定值**。配置 3 使用前调用 `App_ImuSetDeltaCtrl(实际值)`；新比例在 IMU 任务下一轮处理时生效，并丢弃旧样本、半帧及协议统计。配置 2 的加速度换算不依赖此寄存器。

参考：[Epson G365PDC1/PDF1 官方简表](https://www.epsondevice.com/sensing/ja/pdf/m-g365pdc1_pdf1_briefsheet_j_rev20230630.pdf)、[Epson G365PDF1 型号换算常量](https://github.com/cubicleguy/py_esensorlib/blob/master/src/esensorlib/model/mg365pdf1.py)。现有 `C:\Users\Tang\Desktop\G365\g366_upper.py` 使用 G366 的加速度、温度和增量速度系数，不适用于 G365PDF1 的浮点换算。
