# H7 位置、速度、航向联合回放测试

2026-10-18：已添加板端持续速度/航向观测更新，以及独立 Flash 数据回放构建。
已通过 Horco CMSIS-DAP v2 在实际 H743 上完成两段日志测试，CPU 480 MHz、I/D Cache 均关闭。
联合更新可持续处理 200 Hz 数据并平均 100 Hz 发布；存在约 12.65 ms 的最长发布间隔，
因此不属于严格每 10 ms 的硬实时保证。详细实测结果见文末。

## 输出和观测的区别

`NavOutput.state.velocity[3]` 本来就输出 NED 速度（m/s），
`state.attitude[3]` 本来就输出 roll/pitch/yaw（rad，航向为第 3 项）。
这次补的是将多普勒速度、双天线航向持续送入滤波器，而不是新增已有的状态字段。
早期测试仅从 Keil Watch 读取结果。后续已实现独立串口 80 字节发送器，新回放目标经 USART3 发带 SIMULATED 位的结果，见文末更新。

公共入口 `Nav_PushObservation`，位标志为 `NAV_OBS_POSITION/VELOCITY/HEADING`。
同一 GPS 历元先合并，再投递一次；允许仅速度、仅航向或任意组合。
位置 rad/rad/m，速度 NED m/s；航向是主到副天线的导航系基线方位，
`baseline_yaw_frd_rad` 是基线在车体 FRD 水平面相对车头的方向。
本次车辆基线朝右，故为 +90°，不是把 GNSS 航向直接当作车头 yaw。

输入层负责 CRC、解状态、新鲜度、基线长度、速度 latency/age 检查。
回放复用 PC 质量筛选：固定解航向、≤5° 标准差、基线长度检查，
有效多普勒速度、水平/垂直标准差上限 1/2 m/s、latency/age 为零。
板端内核再执行速度 sigma 下限 0.05 m/s、航向下限 0.5°，NIS 门限 16.27/10.83。
近竖直基线拒绝航向更新；位置、速度、航向共享每历元一次误差反馈。
保持完整 21 状态、double、逐帧协方差传播、Joseph 更新、时间插值与杆臂补偿。

`NavStats` 新增位置/速度/航向更新数、速度/航向拒绝数及最近一次 NIS。
`gnss_accepted/gnss_updates` 统计合并观测历元，不等于某一种观测的采用次数。
缺测或 NIS 拒绝不等于队列丢帧。API 数据结构已扩展，调用方需要一并重新编译。

## 生成、验证和编译

在工程根目录：

```powershell
# 首次没有离线增强程序时先执行
powershell -ExecutionPolicy Bypass -File tools/build_kf_gins_aided.ps1

# 默认采用 160516；生成数据、对照、主机验证和独立 Keil HEX
powershell -ExecutionPolicy Bypass -File tools/build_h7_replay.ps1

# 第二段数据，仅主机验证；去掉 -SkipFirmware 也可生成该段 HEX
powershell -ExecutionPolicy Bypass -File tools/build_h7_replay.ps1 `
  -Log tools/sample/terramind_20261003_160557.txt `
  -OutputDirectory build-navigation/h7-replay-160557 -SkipFirmware

# 全部导航主机回归
powershell -ExecutionPolicy Bypass -File tools/test_navigation.ps1
```

默认产物目录 `build-navigation/h7-replay/`：

- `navigation_replay.uvprojx`：独立 Keil 工程，路径按当前工作区生成。
- `firmware/navigation_replay.hex` / `.axf`：可烧录固件与调试符号。
- `replay_data.cpp`：const 数据放 Flash，保留 double；不在 H7 解析文本。
- `manifest.json`：源日志/sidecar/参考程序 SHA256、安装参数、种子、时长、延迟。
- `reference_1.nav` / `reference_7.nav`：独立离线算法的仅位置/三类观测对照。

原始日志及配套 JSON 不改写；第二段仅排除文件末尾的不完整 I 行，内部坏记录拒绝导出。
启动补偿、比例、轴向与 PC 一致；导出后不能在 H7 再扣一次启动均值。
初始状态与种子选择沿用现有 PC 桥接器，因此本轮验证是与现有回放一致，
不是独立真值精度验收。时基仍沿用日志的条件 PPS/GPS 关系。

生成工程复制当前采集工程的编译/时钟设置，将 `app_tasks.cpp` 替换为测试入口，
将默认任务改为独立静态 24 KiB 栈；默认采集工程源文件和工程文件不修改。
运行测试固件期间不启动传感器采集/USART6 日志。这是导航计算和 RTOS 调度测试，
尚不包含传感器持续接收、串口结果发送和日志并发负载。
现有外设/TIM2 初始化仍执行，测试硬件应与原工程一致。

## 板端操作和判读

连接目标 H743 的 CMSIS-DAP 或 ST-Link/SWD 并上电，打开生成的 Keil 工程，烧录对应 HEX，
运行后观察 `g_nav_replay_report`。采样期间不要暂停或单步，否则时间统计失真；
结束后再暂停读取，或使用不暂停内核的 Watch。

程序核对 `SystemCoreClock==480000000`，沿用现有 I/D Cache 状态并记录状态。
预留 64 KiB 上下文存储，运行时检查实际 `Nav_ContextSize()` 是否容纳。
使用 DWT CYCCNT 计时，外层扩展 64 位，避免 480 MHz 下约 8.95 秒回绕影响总时长。

`phase`：0 启动，1 尽快回放，2 实时回放，3 完成，4 故障。
**3 只表示运行完成，不能单凭它认定性能达标。** `error` 为 `NavStatus`。

- `fast`：虚拟测量/到达时间按 1 ms 推进，不等待；`processing_hz` 为全流程吞吐率。
- `realtime`：按 DWT 实际时间投递，IMU 到达时间来自 T，合并观测到达时间取相关 N/R 的最晚值。
  默认 200 ms 处理缓冲，实际计算完成后取发布时间，不用计算前时刻冒充发布时刻。
- `stats.total_process_cycles/imu_processed` 是平均单帧周期，除以 480 得微秒；
  `max_process_cycles`、`branch_max_cycles[4]` 给出预测/前端点/后端点/插值路径最大值。
- `branches_over_5ms` 是最大耗时超过 5 ms 的分支种类数，不是超时帧数。
- `output_hz` 按第一到最后一次发布的真实时间计算；fast 的发布时钟是虚拟时间，
  只有 realtime 的该字段用于实际输出频率验收。
- `stats.imu_queue_peak`、`late_gnss`、溢出、`skipped_output_slots`、`max_output_age_s`、
  `max_publish_interval_s` 和 `stack_free_bytes` 用于检查稳定性及延迟。
- `last_output.state.velocity/attitude` 可直接查看最终速度和航向，航向单位 rad。

200 Hz 对应 5 ms / 2,400,000 cycles。要求全段处理能力超过 200 Hz，并保留余量；
100 Hz 发布平均间隔约 10 ms，不能靠重复发送同一序号凑频率。
短时某分支超过 5 ms 是否可接受，需结合端到端延迟与队列峰值评估，不能只看平均数。
保持 200 ms 缓冲时，100 Hz 发布仍会带约 200 ms 状态年龄。
后续接入实时采集和串口发送后必须再测；开启 Cache 或改变优化级别也必须记录并重测。

## 已完成的验证

主机测试覆盖 7 种观测组合、3 种更新时刻，与未改动的离线增强实现逐帧比较状态和完整协方差；
另验证 Jacobian、旋转杆臂、航向跨零、异常观测门限、近竖直基线拒绝，以及旧仅位置路径。

| 实测日志 | 处理帧 | 模拟 100 Hz 发布 | IMU 队列峰值 | 速度采用 | 航向采用 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 160516 | 5500 | 2750 | 40 | 269 | 259 |
| 160557 | 3292 | 1646 | 40 | 158 | 160 |

两段均通过仅位置、三类观测逐帧 PC 对照，以及记录到达时间的延迟队列回放；
无迟到、溢出或发布时隙跳过。以上发布数是主机虚拟时间测试结果，不能作为 H7 实测速率。

## 2026-10-18 板端实测

目标 DBGMCU ID `0x20036450`、Flash 2048 KiB，探针 `13191edf`。
独立测试固件运行时确认 CPU 480,000,000 Hz，上下文 57,728 字节。
未启用 I-Cache/D-Cache，沿用原工程编译设置和时钟；每段均执行最快回放和真实节拍回放。
烧录后对 HEX 所含地址逐字节回读校验，运行中仅读取 phase，不暂停 CPU。
报告结构的大小及成员偏移由 ELF DWARF 校验（ARM 使用短枚举，不能直接套用 PC ABI）。

| 指标 | 160516 | 160557 |
| --- | ---: | ---: |
| 最快全流程吞吐 / 帧每秒 | 305.995 | 305.898 |
| 实时发布 / Hz | 99.99992 | 99.99987 |
| 实时处理帧 / 发布次数 | 5500 / 2750 | 3292 / 1646 |
| 实时平均单帧计算 / ms | 3.2354 | 3.2366 |
| 实时最大单帧计算 / ms | 10.5092 | 10.5065 |
| 实时最长发布间隔 / ms | 12.6487 | 12.6460 |
| 实时 IMU 队列峰值 / 帧 | 41 | 42 |
| 栈剩余 / 字节 | 8188 | 8188 |
| 迟到、溢出、数值故障、发布跳时隙 | 全部 0 | 全部 0 |

两段实际观测都走帧内插值更新；第一段纯预测分支最大约 2.881 ms，
插值联合更新最大约 10.509 ms。端点更新分支在这两段实测中没有出现，
其正确性有合成测试覆盖，但不把零计数的耗时当作测得的上界。
按 200 Hz 输入计算，核心平均占用约 65% 的 CPU 时间，尚有平均余量；
一次联合更新超过两个 5 ms 输入周期，队列会短暂积压。因此平均吞吐达标，
后续要求严格 10 ms 输出或叠加实际采集/串口负载时，应继续测调度和优化。

真实节拍报告中的 `processing_hz` 把初始 200 ms 缓冲及末尾排空计入总运行时长，
因此分别约 198.53、197.56；这不是漏处理 IMU。全部处理计数与夹具一致，
发布频率则按第一至最后一次发布计算。第一段最大结果年龄约 210.81 ms。

两段最终状态与独立 PC 增强参考的差异分别小于：位置 0.2 μm、速度 0.2 μm/s、姿态 4e-9 rad。
这是跨平台数值一致性检查，不是实际定位精度。速度 NIS 拒绝分别 5/6 次，航向 NIS 拒绝 0 次；
质量筛选未通过的航向没有进入板端队列，不应与 NIS 拒绝混淆。

原始实测文件：

- `build-navigation/h7-replay/board/measurement.json` 与 `.bin`
- `build-navigation/h7-replay-160557/board/measurement.json` 与 `.bin`
- `build-navigation/h7-replay/board/backup.json`：原 2 MiB Flash 备份及 SHA256
- `build-navigation/h7-replay/board/restore.json`：恢复原程序后的全 Flash 回读校验

`tools/run_h7_replay.py` 可用已安装 pyOCD 的 Python 运行，需要显式探针 ID 和已校验的备份清单。
本机 CMSIS-DAP 的硬件 reset 状态查询不可用，使用 attach 连接、1 MHz SWD、SYSRESETREQ 软件复位。
禁止自动解除读保护；使用扇区写入并保留未覆盖区。该脚本不会自动恢复，重复运行后需恢复原备份，
或明确选择保留测试固件。上述恢复原采集程序是早前测试的历史记录。

## 后续实时接入更新

默认工程现已接入导航任务与 USART3 结果发送器，最新部署及 COM11 物理发送验证见
[上车测试说明](navigation_vehicle_test.md)。新生成的回放目标在实时阶段也发送带 SIMULATED 位的结果，
报告新增串口计数、最大服务周期及最后一帧；它仍不启动真实导航任务。测试后应重新烧录正常工程。
