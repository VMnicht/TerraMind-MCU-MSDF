# G365PDF1 UART4 burst 解包

UART4 使用 230400 baud。传感器已切到增量输出，STM32 现在在 `TerraMind_FW/App/imu_capture_config.h` 中默认选择 `APP_IMU_MODE_DELTA32`，初始化收包前应用。预期频率为 200 Hz。`DLT_CTRL` 默认改为待配置的 `0x0048`（角增量代码 4、速度增量代码 8）；必须先通过配置上位机写入并实际读回确认，后续实测不同应修改此处或调用 `App_ImuSetDeltaCtrl(实际值)`。这些配置和 `App_ImuSetMode` 只切换 STM32 的解包方式，不写传感器寄存器；原始模式 2 的接口仍保留。

## 上电静止标定与输出门控

上电后继续接收解析 IMU/GNSS，但 USART6 保持静默，连 `V` 版本头也不发送。首个有效 DRDY 样本建立时间基准，随后连续累计 `IMU_GYRO_CALIBRATION_SECONDS`（默认 3 秒，约 600 帧）有效静止数据。标定完成才开启 USART6，先发 `V`/`B`，再发送后续原始数据。启动统计窗口不补发；请保持静止到上位机出现数据，而不是只按电源接通后的 3 秒计时。采集器应先连接，允许静默期间开始保存。

`gyro_bias_calibration` 使用固定内存、双精度角增量累计和逐帧方差统计；角速度均值为 `sum(delta_angle)/sum(dt)`，dt 来自 TIM2 DRDY，标定与 `T` 日志共用一次匹配。模式 2 仍支持，先按实际 dt 转成内部增量。COUNT/DRDY 不连续、dt 不在 4–6 ms、FLAG 错误/超量程会重新统计。默认门槛：三轴角速度绝对值不超过 5 °/s、标准差不超过 0.15 °/s，0.5 秒分块均值的最大差不超过 0.03 °/s；比力模长距标准重力不超过 0.15 g，各轴标准差不超过 0.03 g，窗口温差不超过 2 °C。有新鲜有效 GNSS 速度时，水平或垂直速度超过 0.2 m/s 也会重置窗口。这些门槛是初始工程参数，需实测调整；常值慢转仍可能与零偏混淆，操作者保持静止是必要条件。

标定结果在本次启动内冻结，不在车辆运动时持续学习。切换解包模式或 DLT_CTRL 后中止旧发送队列，重新静默标定。静默期间继续排空输出用的 DRDY/PPS FIFO，保留匹配历史与 PPS/GPS 配对，避免等标定时队列溢出。

Keil Watch：`g_gyro_calibration_debug.state` 为 0 等待、1 统计、2 完成；`samples`/`duration_s` 为进度，`restarts` 为重置次数；`reason` 为 0 无错误、1 无效样本、2 时间/丢帧、3 运动/重力检查失败、4 方差/分块漂移/温度检查失败。若始终无串口数据，先看该状态、DRDY 配对、FLAG、传感器模式和比例设置。当前固件默认按 DLT_CTRL=0x0048 换算；必须匹配传感器实际设置，固件不写传感器寄存器。

`App_ImuGetLatest`、`g_g365_debug_sample` 和 `I` 记录保留原始量。`App_ImuGetCorrected` / `g_imu_corrected_debug` 提供 `delta_angle_raw_SI - stationary_rate * dt` 的独立调试结果，速度增量不改；使用前检查 `valid` 和时间。这里减去的是静止均值，包含地球自转，仅用于相对静止的显示/调试，不能直接当作 KF-GINS 输入。`App_ImuGetCalibration` 返回均值、逐帧标准差、比力均值与温度。

未来接入导航时，先将均值/标准差转换到前右下轴系（标准差仅换轴、不变符号），用标定时的纬度和姿态调用 `Nav_InitialGyroBiasFromStatic`，去除地球自转投影后填入 `NavConfig.initial.gyro_bias`，仍给算法原始增量，由原有 EKF 统一补偿。不能同时再减静止均值。该辅助接口尚未接入实际导航流程，且不能以移动后的姿态代替标定时姿态。

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

G365PDF1 的两个增量代码位于 `DLT_CTRL` 低字节；STM32 不主动读取该寄存器，目前默认 `0x0048`，要求用户先配置传感器；这不是 MCU 自动读回。更换设备或调整量程后，配置 3 使用前调用 `App_ImuSetDeltaCtrl(实际值)`；新比例在 IMU 任务下一轮处理时生效，并丢弃旧样本、半帧及协议统计。配置 2 的加速度换算不依赖此寄存器。

参考：[Epson G365PDC1/PDF1 官方简表](https://www.epsondevice.com/sensing/ja/pdf/m-g365pdc1_pdf1_briefsheet_j_rev20230630.pdf)、[Epson G365PDF1 型号换算常量](https://github.com/cubicleguy/py_esensorlib/blob/master/src/esensorlib/model/mg365pdf1.py)。现有 `C:\Users\Tang\Desktop\G365\g366_upper.py` 使用 G366 的加速度、温度和增量速度系数，不适用于 G365PDF1 的浮点换算。

## 2026-10-02 增量比例修正

上位机实测：200 Hz，BURST_CTRL1=0xCC07，BURST_CTRL2=0x0C00，ATTI_CTRL=0x0200，DLT_CTRL=0x0008。此前按 0x00CC 解释导致角增量额外放大 4096 倍、速度增量额外放大 16 倍。现同步板端、采集 GUI 与 KF-GINS 桥接工具的默认值，保留 G365PDF1 基础系数。

用户提供的 23 帧 HEX 校验全部通过，COUNT 交替增加 312/313。第一帧按实测配置换算：Y 角增量约 0.020000865°，Z 速度增量约 -0.049435254 m/s。旧日志的原始整数无需重写；离线分析必须使用采集当时实际量程，已有 sidecar 不自动改写。若旧 B 记录声明了错误的比例，保留冲突提示，不将它当作新比例下的有效标定。

## 2026-10-03 动态范围配置

角代码 0 在 200 Hz 下只能表示约 ±49.65°/s，已在实采转弯中削顶。新固件和新采集默认采用角代码 4 / 速度代码 8，即 `0x0048`，但必须配置传感器并重新烧录匹配固件后使用。操作步骤见 [增量比例配置说明](../tools/g365/增量比例配置说明.md)。旧样本仍按其 B/JSON 中的 `0x0008` 解释。
