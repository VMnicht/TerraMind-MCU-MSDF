# USART6 监视数据记录

USART6 的 STM32 TX 为 PC6，接 3.3 V TTL 串口转接器的 RX，并共地。PC 串口助手设置为 **921600、8N1**，保存原始文本，行结束为 CRLF。UART6 发送只调用 `HAL_UART_Transmit_DMA()`；DMA1 Stream7 与 USART6 中断由 CubeMX 配置。8 KiB 固定队列避免串口忙时阻塞 IMU/GNSS 接收任务。若队列满，丢弃整条记录并增加 `records_dropped`，不会输出半条新记录。

每条记录以单字母类型开头：

USART6 在启动静止标定完成前不发送任何记录。默认要求首个有效时间基准之后连续 3 秒静止数据；启动期间的数据不补发。完成后输出 `V` 和 `B`，随后恢复下面的传感器记录。`B` 每秒重复一次，支持上位机晚于启动连接；没有混入普通调试文本。需要使用已支持 `B` 的新版采集器，否则旧采集器会将其报告为未知行。

| 类型 | CSV 字段 | 含义 |
| --- | --- | --- |
| `V` | `V,2,921600,4000000` | 日志格式版本、监视口波特率、TIM2 标称 tick/s，启动时一条 |
| `B` | `B,1,mcu_ms,mode,delta_ctrl,samples,duration_us,mean_x,mean_y,mean_z,std_x,std_y,std_z,temperature_mc` | 启动静止统计结果；版本 1，均值/逐帧标准差单位为纳弧度/秒（乘 1e-9 得 rad/s），原始传感器 XYZ 轴；温度毫摄氏度。均值包含地球自转，I 行没有预减此值 |
| `I` | `I,mcu_ms,mode,count,flag,temp_raw,gpio,x1,y1,z1,x2,y2,z2` | 每个通过 G365 校验的 IMU 帧；模式 2 的两组 32 位有符号整数为角速度、加速度；模式 3 为增量角、增量速度 |
| `N` | `N,mcu_ms,原始消息` | UART5 收到的每条完整原始行，包括 `BESTNAVA`、`UNIHEADINGA`、`GNGGA`、`THS` 及校验失败的行；保留 GNSS 消息自身的时间和校验字段 |
| `S` | `S,mcu_ms,imu_frames,imu_crc,imu_framing,imu_count_jumps,imu_rx_overflow,imu_uart_errors,imu_rearm_errors,gnss_bestnav,gnss_heading,gnss_crc,gnss_format,gnss_line_overflows,gnss_rx_overflow,gnss_uart_errors,gnss_rearm_errors,tx_dropped,tx_discarded_bytes,tx_dma_start_errors,tx_uart_errors,tx_queue_high_water,format_errors` | 每秒累计计数，用于找接收错误、DMA 故障和监视口丢记录 |
| `D` | `D,seq,mcu_ms,tick_hi,tick_lo` | 每个 IMU DRDY 的 TIM2 CH1 硬件捕获 |
| `P` | `P,seq,mcu_ms,tick_hi,tick_lo,period_ticks` | 每个 GNSS PPS 的 TIM2 CH2 硬件捕获；第一个周期为 0 |
| `T` | `T,mcu_ms,imu_count,drdy_seq,drdy_hi,drdy_lo,uart_end_hi,uart_end_lo` | 紧跟对应 `I` 行；有效帧配对 DRDY 及 UART 最后一个字节的时刻；未配对时 DRDY 字段为 0 |
| `R` | `R,mcu_ms,uart_end_hi,uart_end_lo` | 紧跟对应 `N` 行；原始 GNSS 行末字节进 UART5 的时刻 |
| `H` | `H,mcu_ms,timer_hz_nominal,drdy_captures,pps_captures,drdy_queue_drops,pps_queue_drops,drdy_overcaptures,pps_overcaptures,imu_unmatched` | 每秒累计硬件捕获、溢出和配对诊断 |
| `A` | `A,mcu_ms,pps_seq,gps_week,gps_tow_ms,pps_hi,pps_lo,period_ticks,serial_lag_ticks,consecutive,locked` | 整秒 BESTNAVA 与 PPS 的候选配对；连续 3 秒一致后 `locked=1` |

`mcu_ms` 仍是 `HAL_GetTick()` 毫秒诊断值；精确边沿请用 `D`/`P` 的 `(tick_hi << 32) | tick_lo`。TIM2 的 32 位计数器溢出由中断扩展到 64 位，标称 4 MHz（0.25 µs/tick），实际频率由相邻 PPS 的 `period_ticks` 测得。`T` 的 UART 末字节时间来自 UART4 接收中断，`R` 来自 UART5 接收中断。`N` 中的 GPS 周/TOW 是 GNSS **测量历元**，不是 UART 输出时刻。`I` 中的原始整数换算系数见 `g365_uart_modes.md`。

TIM2 CH1 为 PA15（G365 DRDY），CH2 为 PB3（UM982 PPS）。当前 CubeMX 将两路都设为**下降沿**；PPS 有效沿需按 UM982 实际配置和波形确认，DRDY 也应核对。PB3 同时是 SWO，使用 PPS 时不能再把该脚用于 SWO 跟踪。保持 SWD 调试即可。TIM2 初始化后软件启动双通道中断捕获和溢出中断；启动时按实际 APB1 分频、TIMPRE 和 PSC 校验 TIM2 必须精确为 4 MHz，并检查 PSC/ARR 寄存器与全范围 32 位向上计数模式；配置错误会在启动阶段报错（`g_boot_stage=0xB00700E3`），避免悄悄写出错误时间比例。

UM982 官方命令手册规定 `CONFIG PPS` 的 `POSITIVE` 以**上升沿**、`NEGATIVE` 以**下降沿**对齐整秒，且 PPS 时间基准可选 GPS/BDS/GAL/GLO。`BESTNAVA` 头使用 GPS 周/TOW，因此本方案要求 PPS 使用 **GPS 时间基准**。当前 MCU 的下降沿配置要求 UM982 使用 `NEGATIVE`；若 UM982 配的是 `POSITIVE`，请把 CubeMX TIM2 CH2 改为上升沿再生成代码。以示波器或逻辑分析仪核对边沿和脉宽；仅看每秒一个脉冲不能判断捕获的是整秒起点还是脉冲结束。官方手册：<https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.6.pdf>，第 4.3 节。

PC 采集仍保存**原始文本**。固件收到新的整秒 `BESTNAVA` 后，也会把最近 PPS 与 GPS 周/TOW 条件配对；连续 3 秒关系一致时 `A` 行的 `locked=1`，并提供 `TimeSync_GpsToTicks()` 供后续 MCU 融合使用。若超过 2.5 秒没有 PPS，查询会失锁。运行 `python tools/time_sync_report.py 日志.txt --csv imu_gps_time.csv` 可独立检查 PPS 周期、DRDY 与 UART 帧尾延迟、配对率并导出 IMU 的条件 GPS 时间。固件和离线工具都要求对应整秒 `BESTNAVA` 消息在 PPS 后 300 ms 内到达；该门槛还可挡住宽度约 500 ms 的正脉冲下降沿被误认成整秒。仅凭这组串口消息仍无法排除整整 1 秒的归属错误。缺少配对时 CSV 的 GPS 时间留空，保留原始 TIM2 tick。`tools/fusion_replay.py` 在 PPS、GPS 整秒配对及至少 98% 的 IMU/DRDY 配对都成立时采用硬件时基，否则退回旧版到达时间拟合并明确提示。

Keil Watch 可看 `g_monitor_started`（标定时为 0，完成后为 1）、`g_gyro_calibration_debug`、`g_monitor_debug_tx.records_dropped`（理想为 0）、`g_monitor_debug_tx.queue_bytes`、`g_monitor_debug_tx.uart_errors` 和 `g_monitor_format_errors`。目前工程没有启用 D-Cache；若之后启用，需在 DMA 发送前清理对应的缓存行。

上板后先静止采集至少 30 秒：每秒应约有 200 条 `D`、200 条 `I`/`T`、1 条 `P`，连续 PPS 的 `period_ticks` 应接近 4,000,000（时钟源误差会反映在偏差中）。`T` 的 `drdy_seq` 应非 0，且 UART 帧尾减 DRDY 一般为正且稳定；`A` 在连续 3 个候选整秒后应出现 `locked=1`。检查 `H` 的两个队列丢失、两个过捕获和 `imu_unmatched`，以及 `S` 的 `tx_dropped` 是否保持不变。若 `P` 有 1 Hz 但 `A` 无法锁定，优先核对 UM982 PPS 极性、GPS 时间基准和 BESTNAVA 输出延迟。

`I` 在 G365 完整帧通过校验后入队，`N` 在 UART5 收到完整换行后入队，DMA 空闲时由当前任务启动；DMA 完成回调会立即续发队列中的下一段。若 UART6 跟不上输入，整条新记录被拒绝，`tx_dropped` 增加。DMA 故障后待发字节会被清空，`tx_discarded_bytes` 增加。检查相邻 `S` 记录的计数和 `tx_queue_high_water`，才能判断某次采集是否发生固件侧丢失；PC 串口助手自身的丢失仍需由文件中的 G365 COUNT 和 GNSS 历元连续性检查。

## 240 MHz AHB 时钟配置

当前工程使用 HSE=25 MHz，PLL1 M=5、N=192、P=2，CPU/SYSCLK=480 MHz；AHB `/2` 后 HCLK=240 MHz，APB1/APB2 `/4` 后 PCLK=60 MHz。在 TIMPRE=0 时，TIM2/TIM8 输入时钟为 120 MHz。板上实际晶振必须与 HSE_VALUE=25 MHz 一致；如果目标是 CPU=240 MHz，应重新配置 PLL/CPU 分频，再按实际 TIM2 输入时钟选择 PSC。

CubeMX 的 TIM2 PSC 设置为 **29**，即 `120 MHz / (29 + 1) = 4 MHz`。修改时同时保存 `.ioc` 并生成 `Core/Src/tim.c`；ARR 保持 `0xFFFFFFFF`。旧配置 `84 MHz / (20 + 1)` 也可通过校验。TIMPRE=1 时，APB `/1`、`/2`、`/4` 的定时器输入为 HCLK，其他分频为四倍 PCLK，必须重新选择 PSC；例如当前 APB1 `/4` 时需要 PSC=59 才能保持 4 MHz。初始化不会自动改写 PSC。

固件共用 `TIME_SYNC_TIMER_HZ=4000000` 生成 V 行和 H 行标称频率，并推导 PPS 周期容差及配对/超时窗口。日志版本仍为 V2，现有上位机、时间报告和融合回放的 4 MHz 换算保持兼容。PPS 实测周期继续用于 GPS 时间映射，CPU 升频不改变传感器历元或采样周期。

TIM8 的 HAL 毫秒时基也按 APB2 分频及 TIMPRE 计算输入频率；当前 PSC=119、ARR=999，仍为 1 ms。FreeRTOS 通过 SystemCoreClock 配置 1000 Hz tick；当前 CPU 配置下 SystemCoreClock 应为 480000000，不能填成 AHB 的 240000000。CubeMX 再生成时需检查 TIM8 时基中的分频判断是否被覆盖。

主机验证可运行 `powershell -ExecutionPolicy Bypass -File tools/test_navigation.ps1`，其中同步测试覆盖新旧时钟树、TIMPRE/APB 分频、错误 PSC/ARR/计数模式、PPS 门槛、10 ms 配对窗口、2.5 s 失锁和 32 位回绕，日志测试检查 V2 输出兼容性。Python 回归可运行 `python -m unittest discover -s tests -p "test_*.py"`。

上板验收先静止采集 30–60 秒，确认 PPS 约 4000000 tick/s、200 Hz DRDY 相邻间隔约 20000 tick、A 行在连续三个有效候选整秒后锁定，丢队列及过捕获计数不增长。再持续采集超过 18 分钟：4 MHz 的 32 位计数器约每 17.9 分钟回绕一次，64 位时间应连续。可用 `python tools/time_sync_report.py 日志.txt --csv imu_gps_time.csv` 检查时间记录。主机测试及固件编译不能替代实际 PPS 极性、GPS 时间基准、晶振和时基波形验收。
