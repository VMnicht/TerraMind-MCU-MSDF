# IMU 配置上位机

这里保存本次使用的 `C:\Users\Tang\Desktop\G365` 上位机脚本和启动入口，方便随固件版本复现。Windows 下安装带 Tkinter 的 Python，双击 `启动上位机.bat` 或运行 `python g366_upper.py`。

设备连接成功后，右侧“设备配置（实际读回）”显示 BURST_CTRL1/2、ATTI_CTRL、DLT_CTRL/GLOB_CMD3 和两个增量比例代码；点击“复制配置”可复制。仅监听、无法读取寄存器时会提示配置未知，不把默认比例显示为实测值。

此脚本保留原有 G366 型号名称和换算逻辑。本次只增加实际配置显示与复制功能；G365PDF1 的固件/离线换算以 `../capture_profile.py` 和 `../../doc/g365_uart_modes.md` 为准。2026-10-02 设备实际读回 DLT_CTRL=0x0008，角增量代码 0，速度增量代码 8。

参数读取需要上位机独占 IMU 串口。模式切换和 Flash 保存是独立操作；固件 USART6 原始数据采集请使用 `../monitor_gui.py`。
