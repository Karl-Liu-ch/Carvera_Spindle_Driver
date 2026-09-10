# Carvera 主轴驱动实验项目

本仓库包含多套 Arduino 固件和 ODrive 台架脚本，用于把 Carvera 的主轴转速指令转换为不同主轴驱动方案的控制信号。各目录是**互斥的备选实现**，仓库没有声明哪一套是正式生产版本。

## 安全须知

本项目可能驱动高速主轴，并涉及危险的机械运动、电气能量和回生能量。

- 调试时拆除刀具和工件，固定主轴，并确保独立急停可用。
- 改线前断电；固件报警和停机逻辑不能代替外部安全回路。
- 提升转速或修改 `DIRECTION_SIGN` 前，必须先在极低转速验证方向。
- 根据实物核对控制器额定值、电流限制、母线电压、制动/回生吸收能力以及电机和编码器配置。
- CAN 总线必须使用合适的收发器、共地，并在两个物理末端各接 120 Ω 终端电阻。
- 不同电压域之间应电平转换或隔离。ESP32-C6 GPIO 为 3.3 V，不能耐受 5 V。

源码中的参数是当前项目假设，不是经过认证的设备额定值。

## 目录说明

| 路径 | 用途 |
| --- | --- |
| `ODRIVE_NANO_R4_CAN_v3_HALL/` | Nano R4 通过 CANSimple 控制 ODrive S1，使用 Hall 反馈，目标固件 0.6.11。 |
| `ODRIVE_NANO_R4_CAN_HALL_v4/` | 另一套 Hall/ODrive 分支，增加启动失败监测。 |
| `ODRIVE_NANO_R4_CAN_v3/` | 增量编码器 ODrive 分支，目标固件 0.6.12，提供 USB `index_search` 命令。 |
| `ODRIVE_NANO_R4_CAN_v1/`、`v2/`、`ODRIVE_NANO_R4_CAN/` | 保留用于比较或特定硬件的早期 ODrive 版本。 |
| `SOLO_NANO_R4_CAN_v2/` | Nano R4 通过 CANopen 控制 SOLO PICO，含 Nano R4 兼容源码。 |
| `CARVERA_ESCON_INTERFACEBOARD/` | Nano R4 将 PWM 转换为 ESCON 类模拟转速输入。 |
| `ESP32_C6_PWM_TEST/` | ESP32-C6 指令信号发生器，仅用于有人监督的台架测试。 |
| `Odrive.py` | 在 `odrivetool` 会话中配置 ODrive S1 与 AS5047P SPI 编码器的脚本。 |
| `odrivetest.py` | 会实际加速电机的交互式 `odrivetool` 台架脚本。 |
| `error.txt` | 一次 Hall/ODrive 运行的串口日志样本，不是测试套件或认证记录。 |

`.arduino-build-*` 是已忽略的生成构建产物，不是源码。仓库没有自动化测试或 CI 配置。

## Nano R4 通用接线

| 信号 | Nano R4 引脚 | 说明 |
| --- | --- | --- |
| Carvera 主轴 PWM 指令 | D2 | 约 1 kHz 输入。 |
| Blue 转速反馈 | D3 | 输出每主轴转 12 个脉冲。 |
| 原生 CAN TX / RX | D4 / D5 | 必须通过正确的 CAN 收发器连接。 |
| Red 报警 | D6 | 较新版本故障时拉低，正常时为高阻；最早 ODrive 版本的行为不同。 |

连接所有必要的信号地。Carvera 接口电气规格和 CAN 收发器具体型号未在仓库中定义，接线前必须从实物资料确认。

## 固件行为

主要控制固件测量并滤波 Carvera PWM 占空比，按主轴/电机传动比映射转速，向控制器发送指令，同时合成转速反馈。较新版本还监测控制器通信、驱动错误、启动失败、持续掉速和估算功率负载；发生故障后输出报警并请求安全停机。

- ODrive Hall v3：传动比 1.635，主轴上限 12,500 rpm，电机上限 8,400 rpm，CAN 节点 0、1 Mbit/s。
- SOLO v2：传动比 1.635，主轴上限 16,000 rpm，电机上限 9,700 rpm，电流限制 8.2 A，回生限制 0.5 A。
- 使用前必须按照所选 `.ino` 文件顶部说明，完成并保存匹配的电机、反馈、控制和 CAN 参数。

较新版本通常支持 USB 串口命令 `status`、`clear_errors`、`help`。SOLO v2 另有 `motor_identification`，增量编码器 ODrive v3 另有 `index_search`。涉及运动或清错的命令会要求 Carvera 指令为零且控制器状态合适。

## 编译

安装 Arduino IDE 2 或 Arduino CLI，再安装所选固件需要的开发板核心和库。每次只打开或编译一个 sketch 目录。

```powershell
arduino-cli compile --fqbn arduino:renesas_uno:nanor4 SOLO_NANO_R4_CAN_v2
```

- ODrive/Nano 固件使用 `Arduino_CAN.h` 和 Renesas 核心提供的 `pwm.h`。
- SOLO v2 需要 SOLOMotorControllers 库；兼容文件记录的是 5.5.0，并通过包含库的私有实现文件来启用 Nano R4 原生 CAN。
- ESP32-C6 测试需要 ESP32 Arduino 开发板包，并启用 **USB CDC On Boot**。该固件会故意以相当于 7,500 rpm 的指令启动，在完全理解或修改此行为前只能连接安全的测量装置。

仓库没有保存上传端口。应先选择实际开发板和端口、完成编译并检查警告，再核对所选源码头部的接线与配置说明，最后上传。

## ODrive 脚本

`Odrive.py` 假定当前处于 `odrivetool` 会话并已定义 `odrv0`。它会保存配置并重启驱动器。脚本默认关闭 CAN，因此要配合 CAN 固件使用，之后还必须配置匹配的 CAN 参数。

`odrivetest.py` 同样依赖 `odrv0`；它会改变控制器/滤波状态，并把电机逐步提升到 80 turns/s。它是有人监督的台架流程，不是普通的独立命令行程序。

## 已知限制

- 没有声明正式固件、发布流程、自动化测试或许可证。
- 各分支的控制器固件和反馈硬件不同，不能直接互相复制配置。
- SOLO 调试说明中的 CANopen 节点 ID 1 与源码参数 0 可能是库采用零基编号，但仓库内没有证据可确认。
- 日志只表明某一次运行在接近 12,000 rpm 时表现稳定，不能证明其他工况或硬件已经验证。

英文说明见 [README.md](README.md)，面向 Codex/维护者的简要上下文见 [codex_context.md](codex_context.md)。
