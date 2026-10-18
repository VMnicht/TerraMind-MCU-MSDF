# H7 上车测试与导航结果上位机

2026-10-18：保留 CPU 480 MHz、IMU 200 Hz、导航发布 100 Hz。默认 Keil 工程已接入真实传感器，正常启动不需要电脑下发“开始融合”指令。结果口采用 [80 字节协议 v1](navigation_output_protocol_v1.md)。

## 接线与启动

| 用途 | H7 引脚 | 参数 |
| --- | --- | --- |
| G365 数据输入 | UART4 RX / PA1；TX / PA0 | 230400，8N1 |
| G365 DRDY | TIM2 CH1 / PA15 | 当前捕获下降沿 |
| UM982 数据输入 | UART5 RX / PB12；TX / PB13 | 115200，8N1 |
| UM982 PPS | TIM2 CH2 / PB3 | 当前捕获下降沿；PPS 使用 GPS 时间基准、NEGATIVE 有效沿 |
| 导航结果输出 | **USART3 TX / PB10 → USB 串口 RX** | **460800，8N1**；当前室外接入 **COM10**（原测试为 COM11） |
| 原始日志输出 | USART6 TX / PC6 → 另一 USB 串口 RX | 921600，8N1；原有文本协议 |

接口共地，使用与板卡电平匹配的 UART 转接器。结果接收不需要向 PB11 发指令。PB3 不同时用于 SWO。实际 PPS/DRDY 极性仍应按接收机设置与波形核对，见 [硬件同步说明](monitor_uart6.md)。

首次安装需要确认并保存传感器配置：G365 增量模式 3、200 Hz、DLT_CTRL=0x0048；UM982 持续输出 10 Hz BESTNAVA 和 UNIHEADINGA，并接入差分数据以获得 RTK。H7 不向传感器写配置或保存 Flash；已有配置应掉电保持。具体步骤见 [G365 配置](../tools/g365/增量比例配置说明.md)。帧格式本身不能区分 G365 模式 2/3，不能依靠收到数据来推断寄存器配置正确。

安装参数集中在 `TerraMind_FW/Navigation/navigation_live.h`：传感器 **+Y 向前、−X 向右、+Z 向下**；主天线相对 IMU 的 FRD 杆臂为 **[0, −0.15, 0] m**；主→副天线沿车右。更换安装后必须改为实测值。

1. 先接好数据、DRDY、PPS 和结果口，打开上位机，再上电。
2. **保持车辆静止，直到显示“导航正常”**。启动需要连续 3 秒有效 IMU 标定、连续 GPS/PPS 配对锁定、同历元的有效 RTK 固定位置/双天线航向和低速 Doppler 观测；无固定解时会持续等待。
3. 初始化使用近期静止比力求横滚/俯仰，按实际基线方向及倾斜解算车头航向；标定均值扣除地球自转投影后作为陀螺初始零偏，核心只接收原始增量，避免重复补偿。
4. 正常后 100 Hz 输出位置、NED 速度、RPY、同历元机体→NED 四元数。默认固定处理滞后 200 ms，正常结果年龄约 200–215 ms，并非“超过 100 ms 就过期”。
5. IMU 丢帧/错误、队列溢出或算法数值故障会锁定故障，停止有效导航；保留最后良好状态、原测量时刻并以约 1 Hz 发无效状态帧。重新静止上电后重建会话。GNSS 暂时中断时可继续惯导推算，状态位会清除近期 GNSS/时间锁定/过期 RTK 信息。

## 结果观察程序

双击 **`tools/start_navigation_monitor.bat`**，选择当前结果串口 **COM10**，点“连接”。端口号取决于转接器，以系统枚举和实际接线为准。首次缺少依赖时在工程根目录执行：

```powershell
python -m pip install -r tools/requirements.txt
python tools/navigation_monitor.py --port COM10
```

窗口提供：

- 水平速度、车头航向、横滚/俯仰、结果年龄和接收帧率。
- 纬经度、椭球高、三轴 NED 速度、w/x/y/z 四元数。
- 局部东/北轨迹、航向指针、最近 30 秒速度曲线。
- RTK 解类型、PPS 锁定、GNSS 近期更新、帧序号、CRC/重同步/缺帧/重复统计。

每次连接自动保存 `captures/navigation/navigation_时间.bin`、`.csv` 和 `.json`。BIN 保留收到的全部字节；CSV 使用协议 SI 单位，包含所有完整校验帧及有效性说明；JSON 记录来源、模式、字节数、SHA256、解析统计和是否正常结束。显示队列拥塞会计数，不丢弃已写入原始记录的数据。异常退出可能留下 `finished=false` 的清单。

无效/过期/非有限值/非单位四元数不会更新轨迹。界面保留的数值明确标为“最近有效结果”。连续 100 ms 没有新的有效测量会显示导航流中断；连续 3 秒没有完整 CRC 帧会显示连接超时。低频状态帧只证明通信存活。重复测量不刷新有效性；重新连接、设备序号倒退或测量时间回退会清空旧轨迹；uint32 序号自然回绕不误判为重启。

“打开记录”可按 100 Hz 节奏查看 BIN，界面标记“记录回放”；它不是对原始接收时间间隔的重现。“演示”及 `--demo` 使用带 SIMULATED 位的合成轨迹并持续显示“模拟数据”。车辆固件从不设置 SIMULATED。

## 运行结构

- 高优先级采集任务保持原始 I/N/T/R 等日志输出；每个 IMU/完整 GNSS 消息经固定容量队列交给单一导航任务。DRDY 只匹配一次，再分发给标定、日志和导航。
- 导航任务使用静态 24 KiB 栈和 64 KiB 上下文存储。GNSS 在统一 TIM2 连续秒轴上映射，等待至历元后 170 ms 合并位置/速度/航向，核心处理滞后 200 ms。每轮最多处理一个 IMU，采集任务可抢占。
- 发送使用 AXI SRAM 中两个 32 字节对齐的 96 字节槽，实际发 80 字节。中断仅置完成/错误标志；正在 DMA 发送的槽不可改写，待发槽只保留最新帧。错误/超时先同步停止 DMA 再回收缓冲区。
- `g_navigation_debug` 提供阶段、故障、队列/更新/计时/发送计数及导航栈余量；`g_capture_debug` 提供 IMU、硬件同步和采集栈余量。阶段 0=等标定，1=等时间/固定解/静止对准，2=运行，3=故障。调试数据不是串口协议的一部分。
- UART4/UART5 接收初始化对中断保持原子性，并按 HAL 的实际接收状态重挂接，修复了实板上发现的“启动瞬间溢出后软件仍标记已接收、一直等不到 IMU”的竞态。

## 已完成验证与上车边界

主机：16 项 C/C++ 测试（含完整核心参考对照、真实日志回放、自动对准、合并观测、UART DMA 错误处理、接收启动异常恢复、标准帧）与 59 项 Python 协议/原日志回归通过。Keil 构建零错误、零警告。上位机支持演示及真实串口收包/保存。

H743 实板、CPU 480 MHz、I/D Cache 关闭，采用 `terramind_20261003_160516.txt` 真实日志并经 PB10→COM11 实际发送：

| 项目 | 结果 |
| --- | --- |
| IMU 处理 / 合并观测 | 5500 / 274 |
| 板端发布频率 | 99.99994 Hz |
| PC 接收 | 2750 / 2750 帧，约 100.02 Hz；测量时间轴约 99.99983 Hz |
| CRC / 缺帧 / 重复 / 重同步丢字节 | 全部 0 |
| 结果年龄 | 203–211 ms |
| 最大发布间隔 | 12.463 ms |
| 核心平均 / 最大处理耗时 | 3.225 / 10.319 ms |
| 打包 + DMA 服务最大耗时 | 7288 cycles，约 15.18 μs |
| DMA 替换 / 错误 / 中止 | 全部 0 |

证据：`build-navigation/vehicle-serial-replay/board/serial_measurement.json`、`com11.bin`、`com11.json`。这些帧设置 SIMULATED，表示实测数据在 Flash 中回放；这项验证覆盖核心和物理发送链路，不能替代真实传感器并发下的完整道路验收。

早期室内启动测试：3 秒、600 帧静止标定成功，UM982 的 BESTNAVA/UNIHEADINGA 按 10 Hz 接收，GNSS CRC/格式/接收溢出/USART6 日志丢弃均为 0；COM11 收到约 1 Hz 的正常等待状态帧。当时 UM982 无定位/航向解，未能覆盖 RTK 初始化条件。

早期室内固件 HEX SHA256 为 `4de28baa8ae1009977cfef7e5886d825b539c5414b021cdaba1863d3d951f6fe`，已由下面的室外修复版本替代。
修复 UART 启动竞态后，连续三次 SYSRESETREQ 重启均恢复 IMU 接收并自动完成静止标定；
其中第三次记录了一次启动串口错误，仍正常恢复，未再卡住。
证据为 `build-navigation/vehicle-boot-recovery.json`。最终调试快照及 COM11 状态帧分别保存于
`build-navigation/vehicle-final-board.json`、`vehicle-final-com11.bin/.json`。

### 室外真实 RTK 修复与验证（2026-10-18）

现场已获得 RTK 固定位置、固定双天线航向和 PPS 锁定，但原初始化将 BESTNAV 的 `solution_age_s` 强制等于 0；实测消息持续为 1.000，导致位置和速度都被拒绝，导航一直停留阶段 1。现将位置解龄期限制为有限的 0–2 s，保留 GPS/PPS 测量历元 180 ms 新鲜度检查。速度独立检查其 latency/age 为零，不再由位置解龄期阻止。回归测试覆盖 1 s 正常初始化和 3 s 拒绝初始化。

字段定义参见 [Unicore 官方手册 BESTNAV 表](https://en.unicorecomm.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.13.pdf)：位置 solution age 与速度 timestamp latency 是独立字段；0–2 s 为本工程质量门限，不作为时间戳偏移量。

修复版本已烧录、逐字节回读验证，HEX SHA256 为 `8e29ff0e2cf0bc823ea9574a3d843bc337d8582e6c14f8a4f05c57e15ec109fd`。真实输入自动完成 3 s 标定与初始化；位置、速度、航向持续更新，IMU/GNSS 队列溢出、数值错误、过期观测和发送错误均为 0。COM10 连续 30 s 测得 2999 个有效帧，全部 SIMULATED=0，接收约 100.037 Hz、测量时间轴约 99.998 Hz；CRC/缺帧/重复/重同步均为 0，年龄 203–208 ms。

证据：`build-navigation/outdoor-diagnosis.json`、`outdoor-fixed-board.json`、`outdoor-running-board.json`、`outdoor-fixed-com10.bin/.json`。调试器重新连接可能影响 DWT 计数，因此后续非停核快照的最大 cycles 不作为耗时结论；频率按实际串口和测量时间轴验收。当前完成静止真实输入链路验证，尚未完成道路动态精度验收。

室外开始行驶前，应确认显示“导航正常”、RTK 固定、PPS 锁定，静止速度合理、车头航向与车体方向一致。先记录一段静止数据，再进行低速直行/转弯，同时保存 USART6 原始数据与当前结果串口数据；检查原始丢帧、滤波队列、年龄、航向连续性与位置跳变。定位精度须用独立真值评估。

## 开发复验命令

```powershell
./tools/test_navigation.ps1
python -m unittest discover -s tests -p 'test_*.py'
python tools/capture_navigation.py --port COM10 --seconds 30 --output build-navigation/vehicle-check
```

`tools/inspect_h7_navigation.py` 使用 pyOCD 和 ELF DWARF 读取真实类型布局，无需停核；`--flash` 会校验原 Flash 备份清单，烧录默认真实输入固件并逐字节回读后复位。测试结束应保留正常工程固件，不保留 `navigation_replay.hex`。本机探针为 CMSIS-DAP `13191edf`，不是 ST-Link。
