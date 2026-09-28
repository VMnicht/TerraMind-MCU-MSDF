# UM982 UART5 接收

本工程用 UART5 接 UM982：PB12 为 STM32 RX、PB13 为 STM32 TX，8N1、115200 baud。G365 仍使用 UART4。`AppTasks_RunImu()` 初始化两路接收，并每轮处理两路串口数据。UM982 侧应输出与 `rtkall.txt` 相同的 `BESTNAVA` 和 `UNIHEADINGA` ASCII 消息；程序不会向 UM982 发送配置命令或差分数据。

`TerraMind_FW/BSP/um982_uart_bsp.cpp` 用中断逐字节接收和 2048 字节环形缓冲区。`uart_rx_callbacks.cpp` 按 UART 句柄分发 HAL 回调。解析器忽略 NMEA 和不认识的 Unicore 消息，按 `#` 重新同步，验证 Unicore ASCII CRC32 后才解析字段。

从任务上下文调用 `App_Um982GetPosition()`、`App_Um982GetHeading()` 读取最新消息。返回 1 只表示已经收到过该类消息；使用观测前还要检查 `time_valid`、`position_valid` / `velocity_valid` / `heading_valid`、`fix_type`、标准差和新鲜度。通过 `sequence` 可判断是否出现新消息。`received_at_ms` 是 UART 数据在任务中处理时的 `HAL_GetTick()`，不是 GNSS 采样时刻；USART6 的 `R` 行另给出原始消息末字节进入 UART5 的 TIM2 时刻。融合应使用 `gps_week`、`gps_tow_ms` 和 PPS/DRDY 硬件捕获记录，具体见 `monitor_uart6.md`。两个消息的周内毫秒相等时属于同一 GNSS 历元。

`Um982Position::height_msl_m` 为平均海平面高，`undulation_m` 为大地水准面起伏；不要直接当作椭球高。速度为地面水平速率、相对真北顺时针的运动方向、向上为正的垂直速率。静止时运动方向不稳定，不应当作载体航向。`Um982Heading::heading_deg` 为主天线指向从天线的真北顺时针角；它与载体前向的安装夹角需单独标定。

`App_Um982GetStats()` 给出成功解析的两类消息数量、CRC/格式/过长行错误，以及接收溢出、UART 错误和重启接收错误。若 UM982 输出频率或消息数增加，需要检查 UART5 的 115200 baud 带宽与这些计数。

组合导航离线采集时，USART6 会输出带 STM32 本地时刻的 IMU 与 UM982 监视记录；格式与接线见 `monitor_uart6.md`。

## 上板快速检查

在 Keil Watch 中加入 `g_um982_debug_flags`、`g_um982_debug_position`、`g_um982_debug_heading`、`g_um982_debug_stats`。串口启动成功后 flags 的 bit0 为 1；接收到两类消息后 bit1/bit2 为 1；最新消息有效时 bit3/bit4/bit5 分别表示位置、速度、航向；bit6/bit7 分别表示最新位置和航向为 RTK 固定解。若看到的始终只是 bit0，先检查 UM982 TX 到 STM32 PB12、共地、波特率和消息配置。若消息数增加但有效位为 0，查看解状态与 `time_valid`。CRC、溢出或 UART 错误计数增加时，优先检查串口质量和输出负载。
