# 飞镖代码库 — STM32F407 RoboMaster 飞镖发射系统

基于 STM32F407VGT6 + FreeRTOS 的 RoboMaster 飞镖发射控制系统。支持全自动/半自动比赛模式，通过裁判系统自定义协议（0x0301）接收飞镖参数，USB CDC 用于调试和手动控制。

---

## 目录

- [硬件概述](#硬件概述)
- [引脚分配](#引脚分配)
- [软件架构](#软件架构)
- [通信协议](#通信协议)
- [比赛模式](#比赛模式)
- [CDC 指令与反馈](#cdc-指令与反馈)
- [裁判系统集成](#裁判系统集成)
- [编译与烧录](#编译与烧录)
- [文件结构](#文件结构)

---

## 硬件概述

| 组件 | 型号 | 说明 |
|------|------|------|
| **主控** | STM32F407VGT6 | ARM Cortex-M4 @ 168MHz |
| **云台电机** | GM6020 | CAN ID 0x1FE, 角度环控制 |
| **摩擦轮 ×4** | fric1~fric4 | CAN ID 0x200, 速度环控制 |
| **升降电机** | lift | CAN ID 0x1FF byte 4-5, 用于推镖 |
| **装弹电机** | load | CAN ID 0x1FF byte 2-3, 堵转检测上膛 |
| **裁判系统** | 官方裁判机 | USART6 @ 115200-8N1 |
| **调试串口** | USB CDC | 虚拟串口, 指令+反馈 |
| **蜂鸣器** | TIM4 CH3 PWM | 播放校歌曲调 |
| **RGB LED** | PH12(R)/PH11(G)/PH10(B) | 运行状态指示 |
| **按键** | PA0 (EXTI0) | 用户按键 |

### 电机 CAN 标识符

| ID | 电机 | 数据字节 | 控制模式 |
|----|------|----------|----------|
| 0x1FE | GM6020 (云台) | byte 0-1 (电流) | 角度环 / 速度环 |
| 0x1FF | load + lift | byte 2-3 (load), byte 4-5 (lift) | 速度环 |
| 0x200 | fric1~fric4 | byte 0/2/4/6 | 速度环 |

---

## 引脚分配

| 引脚 | 功能 | 外设 |
|------|------|------|
| PG14 | TX | USART6 → 裁判系统 |
| PG9 | RX | USART6 ← 裁判系统 |
| PA12 | DP | USB_OTG_FS |
| PA11 | DM | USB_OTG_FS |
| PB5 | CAN2 TX | CAN 总线 |
| PB6 | CAN2 RX | CAN 总线 |
| PH12 | GPIO Out | LED_R |
| PH11 | GPIO Out | LED_G |
| PH10 | GPIO Out | LED_B |
| PA0 | EXTI | KEY |
| PB10 | TIM2 CH3 | CAN 发送定时器 |
| PD12 | TIM4 CH3 | 蜂鸣器 PWM |

---

## 软件架构

### FreeRTOS 任务

| 任务名 | 函数 | 优先级 | 栈 (words) | 职责 |
|--------|------|--------|-------------|------|
| `defaultTask` | `StartDefaultTask` | `osPriorityNormal` | 128 | USB 设备初始化, 空闲循环 |
| `MyTask1` | `MotorUpdate` | `osPriorityLow` | 128 | 电机输出更新、安全监测 (100Hz) |
| `myTask2` | `StartTask2` | `osPriorityIdle` | 128 | **主逻辑**: RunningTask 状态机 |
| `PidTask` | `StartPidTask` | `osPriorityIdle` | 128 | 所有电机的 PID 计算 (~1ms) |
| `CdcTask` | `StartCdcTask` | `osPriorityIdle` | 128 | CDC 反馈数据发送 (~10Hz) |

### 数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                        STM32F407 (主控)                              │
│                                                                     │
│  USART6 ──→ Referee Lib ──→ g_referee (解析) ──→ dartParam_array[] │
│    (裁判系统)         ↓                          ↑                  │
│                 0x0301 参数                    RunningTask 7       │
│                                                                     │
│  USB CDC ──→ CDC_Receive_Callback ──→ RunningTask / MotorSetOutput │
│    (上位机)                                                         │
│                                                                     │
│  CAN2 ──→ motor_array[] ←── MotorUpdate (100Hz) ←── TIM2 IRQ       │
│    (电机)        ↓                                                  │
│              StartPidTask (角度/速度环)                             │
└─────────────────────────────────────────────────────────────────────┘
```

### 电机控制层次

```
Timer2 IRQ (100Hz)
  └─→ CAN_SendMessage() 从 MotorSend 结构体定时发送
        └─→ MotorUpdate 任务: 将 motor.output → MotorSend.data[]
              ├─ 安全保护: 温度/转矩/离线检测
              ├─ 堵转检测 (RunToStall)
              └─ 到达检测 (RunToAngle)
StartPidTask (~1ms)
  └─→ PidCalculate() 根据 motorMode 选择 PID 环
        ├─ angleMode → anglePid
        ├─ speedMode / runToStall / speedTimeMode → speedPid
        └─ torqueMode → torquePid (未使用)
```

---

## 通信协议

### 裁判系统协议 (USART6)

基于 RoboMaster 裁判系统串口协议 V1.9.0，帧格式：

```
SOF(0xA5) + DataLength(2B) + Seq(1B) + CRC8(1B) + CmdID(2B) + Payload + CRC16(2B)
```

#### 已处理命令码

| CmdID | 数据 | 用途 |
|-------|------|------|
| `0x0001` | `ext_game_state_t` | 比赛状态 |
| `0x0003` | `ext_game_robot_HP_t` | 机器人血量 |
| `0x0105` | `ext_dart_info_t` | 飞镖发射信息 |
| `0x0201` | `ext_game_robot_status_t` | 机器人状态 |
| `0x020A` | `ext_dart_client_cmd_t` | 飞镖客户端指令（开门状态） |
| **`0x0301`** | 自定义 | **飞镖发射参数（自定义子协议）** |

#### 自定义 0x0301 子协议 (sub_content_id = 0x0001)

| 偏移 | 大小 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1B | `uint8` | `dart_id` | 飞镖 ID (0~3) |
| 1 | 2B | `int16` BE | `v1Speed` | 摩擦轮 1/3 转速 |
| 3 | 2B | `int16` BE | `v2Speed` | 摩擦轮 2/4 转速 |
| 5 | 4B | `int32` BE | `yaw` | 云台角度 (245000 = 中心) |

---

## 比赛模式

通过宏 `DART_COMPETITION_MODE` 选择上电自动启动的模式：

```c
#define DART_COMPETITION_MODE 0  // 0=关闭(手动), 6=全自动, 7=半自动
```

### RunningTask 任务列表

| Task | 名称 | 说明 |
|------|------|------|
| 0 | Idle | 空闲, 等待 CDC 指令 |
| 1 | 单发立即 | 使用 `dartParam_array[0]` 参数立即发射 |
| 2 | 单发延时 | 延时 10s 后使用 `dartParam_array[0]` 发射 |
| 3 | 播放音乐 | 蜂鸣器播放校歌（非阻塞式, 可打断） |
| 4 | 连发 | 按 `dartParam_array[0~3]` 连发 4 发 |
| 5 | 摩擦轮预热 | 控温至 50°C 后保持 |
| **6** | **全自动** | 使用宏预设参数连发 4 发, 需裁判系统开门信号 |
| **7** | **半自动** | 第 1 发宏预设, 后续等待裁判系统 0x0301 参数 |

### 全自动模式 (RunningTask = 6)

使用 `main.c` 顶部的宏定义预设参数：

```c
#define DART_AUTO_YAW_0   245000   // 第1发云台角度
#define DART_AUTO_V1_0    4000     // 第1发摩擦轮1/3转速
#define DART_AUTO_V2_0    4000     // 第1发摩擦轮2/4转速
// ... YAW/V1/V2_1, _2, _3 同理
```

流程：
1. 等待裁判系统开门 (`dart_launch_opening_status == 0`)
2. 按预设参数顺序发射 4 发
3. 发射完毕后 `RunningTask = 0`

### 半自动模式 (RunningTask = 7)

```c
#define DART_SEMI_FIRST_YAW   245000
#define DART_SEMI_FIRST_V1    4000
#define DART_SEMI_FIRST_V2    4000
```

流程：
1. 等待裁判系统开门
2. 第 1 发使用宏预设参数
3. 第 2~4 发等待裁判系统通过 `0x0301` 下发参数，每收到一包立即发射

### 单发序列 (`DartFireSingle`)

```c
void DartFireSingle(double yaw, double v1Speed, double v2Speed);
```

每发飞镖的完整动作序列（阻塞）：
1. 云台旋转至目标角度 (`MotorRunToAngleBlocking`, 300 RPM)
2. 4 个摩擦轮加速至目标转速 (`speedMode`)
3. 装弹电机堵转上膛 (`MotorRunToStall`, -3000 RPM)
4. 升降电机推镖 3700ms (`MotorRunSpeedTimeBlocking`, 30000 RPM)
5. 摩擦轮停止
6. 升降电机回位 3000ms (`-30000 RPM`)
7. 升降电机堵转归零 (`MotorRunToStall`, -6000 RPM)
8. 装弹电机堵转归零 (`MotorRunToStall`, -3000 RPM)

---

## CDC 指令与反馈

USB CDC 虚拟串口用于调试上位机通信。

### 指令数据包 (上位机 → STM32)

#### 电机控制 (Byte 0 = 0x00)

| Byte | 内容 |
|------|------|
| 0 | `0x00` (Header) |
| 1 | Motor ID (0~6) |
| 2 | Mode (0~7) |
| 3+ | 参数 (大端序) |

Mode 详细：

| Mode | 名称 | 参数 |
|------|------|------|
| 0 | Disable | 无 |
| 1 | Current Mode | Byte 3-4: `int16` |
| 2 | Angle Mode | Byte 3-4: `int16` |
| 3 | Speed Mode | Byte 3-4: `int16` |
| 4 | Torque Mode | Byte 3-4: `int16` |
| 5 | RunToStall | Byte 3-4: `int16` Speed |
| 6 | RunToAngle | Byte 3-10: `double` Angle, Byte 11-12: `int16` Speed |
| 7 | SpeedTimeMode | Byte 3-4: `int16` Speed, Byte 5-8: `uint32` Time(ms) |

#### 系统指令 (Byte 0 = 0x01)

| Byte | 内容 |
|------|------|
| 0 | `0x01` (System Command) |
| 1 | `0x00` = 紧急停止 (alarm_level=3) |
| 1 | `0xFE` = 解除急停, 重新使能所有电机 |
| 1 | 其他 = 设置 RunningTask |

#### 数据设置 (Byte 0 = 0x02)

| Byte | 内容 |
|------|------|
| 0 | `0x02` (Value Set) |
| 1 | `0x00` (dartParam Set) |
| 2 | Dart ID (0~3) |
| 3-10 | yaw `double` 大端序 (实际值 = 接收值 + 245000) |
| 11-12 | v1Speed `int16` 大端序 |
| 13-14 | v2Speed `int16` 大端序 |

### 反馈数据包 (STM32 → 上位机, ~10Hz)

长度 43 字节, 首字节 `0x81`:

| Byte | 内容 |
|------|------|
| 0 | `0x81` (Header) |
| 1 | Motor ID (0~6) |
| 2-3 | Single Angle `uint16` BE |
| 4-5 | RPM `int16` BE |
| 6-7 | Torque `int16` BE |
| 8 | Temp `int8` |
| 9 | Flags (`bit7:Enabled, bit6:Stalled, bit5-0:Mode`) |
| 10-13 | Total Angle `int32` BE |
| 14-20 | (后续 7 个电机的压缩反馈) |
| ... | ... |
| 33 | `has_dart_param` (0/1) |
| 34 | `dart_id` |
| 35-36 | `v1Speed` `int16` BE |
| 37-38 | `v2Speed` `int16` BE |
| 39-42 | `yaw` `int32` BE |

> 注意: 此反馈中的 `0x0301` 参数为非消费式读取，不会干扰 `RunningTask=7` 的参数消费。

---

## 裁判系统集成

### Referee 库架构 (OOP 风格)

通过 `referee_ops_t` 函数指针表实现多态：

```c
typedef struct {
    int (*init)(referee_t *me, const referee_settings_t *cfg);
    int (*get_dart_info)(referee_t *me, ext_dart_info_t *out);
    int (*get_dart_client_cmd)(referee_t *me, ext_dart_client_cmd_t *out);
    int (*get_dart_param)(referee_t *me, referee_dart_param_item_t *out);  // 0x0301
    // ...
} referee_ops_t;
```

### 关键 API

- `Referee_Init(&g_referee, NULL)` — 初始化（使用默认配置, huart6）
- `Referee_StartUartReceive(&g_referee)` — 启动 USART IT 接收
- `Referee_Update(&g_referee, HAL_GetTick())` — 超时检测
- `Referee_GetDartInfo(&g_referee, &info)` — 获取飞镖信息 (0x0105)
- `Referee_GetDartClientCmd(&g_referee, &cmd)` — 获取开门命令 (0x020A)
- `Referee_GetDartParam(&g_referee, &param)` — **消费式**获取飞镖参数 (0x0301)

> `get_dart_param` 是消费式读取：读取后清除 `has_dart_param` 标志，确保每包参数只被使用一次。

### 0x0301 解析位置

在 `Referee.c` 的 `referee_handle_cmd()` 函数中处理：

```c
case ID_student_interactive: // 0x0301
    if (payload_len >= 6 && payload[0] == 0x00 && payload[1] == 0x01) {
        // sub_content_id == 0x0001: 飞镖发射参数
        me->dart.dart_param_item.dart_id   = payload[2];
        me->dart.dart_param_item.v1Speed   = (int16_t)((payload[3]<<8)|payload[4]);
        me->dart.dart_param_item.v2Speed   = (int16_t)((payload[5]<<8)|payload[6]);
        me->dart.dart_param_item.yaw       = (int32_t)((payload[7]<<24)|(payload[8]<<16)|(payload[9]<<8)|payload[10]);
        me->dart.has_dart_param            = 1;
    }
    break;
```

---

## 编译与烧录

### 开发环境

- **IDE**: Keil MDK-ARM (µVision)
- **MCU**: STM32F407VGTx
- **HAL**: STM32Cube FW_F4 V1.27.0
- **RTOS**: FreeRTOS (CMSIS-RTOS v2)
- **USB**: STM32 USB Device Library

### 编译

在 Keil MDK-ARM 中打开 `dart_C.uvprojx`，点击 Build (或使用 VS Code EIDE 插件)。

### 烧录

通过 J-Link / ST-Link 烧录，或使用 VS Code EIDE 插件中的 flash 命令。

### 配置切换

编辑 `Core/Src/main.c` 顶部的宏：

```c
#define DART_COMPETITION_MODE 0  // 0=调试模式, 6=全自动, 7=半自动
```

- **调试模式 (0)**: 上电后等待 USB CDC 指令
- **比赛模式 (6/7)**: 上电自检后自动进入比赛流程

---

## 文件结构

```
dart_C/
├── Core/
│   ├── Inc/                    # 头文件
│   │   ├── main.h              # 主函数头, GPIO 引脚定义
│   │   ├── Referee.h           # 裁判系统对象定义 + API
│   │   ├── referee_protocol.h  # 协议常量 + 结构体定义
│   │   ├── Filter.h            # 卡尔曼/低通滤波器
│   │   ├── can.h / tim.h / usart.h / gpio.h
│   │   └── FreeRTOSConfig.h
│   └── Src/                    # 源文件
│       ├── main.c              # 主程序: 初始化, RunningTask, CDC回调
│       ├── Referee.c           # 裁判系统协议解析
│       ├── freertos.c          # FreeRTOS 任务创建
│       └── ...
├── USB_DEVICE/                 # USB CDC 设备栈
│   └── App/
│       ├── usbd_cdc_if.c       # CDC 收发实现
│       └── usb_device.c        # USB 设备初始化
├── Drivers/                    # HAL 驱动
├── Middlewares/                 # FreeRTOS, USB 中间件
├── MDK-ARM/                    # Keil 项目文件
│   └── dart_C.uvprojx
├── CtrlPanel CN.py             # 调试上位机 (中文)
├── CtrlPanel EN.py             # 调试上位机 (英文)
└── README.md
```

---

## 调试上位机

项目附带两个 Python 调试脚本：

- `CtrlPanel CN.py` — 中文版控制面板
- `CtrlPanel EN.py` — 英文版控制面板

通过 USB CDC 连接后，可实时监控电机状态、设置 RunningTask、调整飞镖参数等。

---

## 安全保护

- **温度保护**: 电机温度超过 `maxTemp` 时自动禁用
- **转矩保护**: 电机转矩超过 `maxTorque` 时自动禁用
- **离线检测**: 100ms 未收到 CAN 反馈则判定离线并禁用
- **上电预热**: 电机重新上线后 800ms 保护期
- **紧急停止**: CDC 指令 `0x01 0x00` 立即禁用所有电机

---

## 注意事项

1. **半自动模式下**，第 1 发使用宏定义预设值，后续 3 发需服务器通过裁判系统下发 `0x0301` 参数。
2. **全自动模式下**，所有参数在编译时通过宏定义固化，不依赖裁判系统数据（仅依赖开门信号）。
3. **USB 仅在调试时使用**，比赛场上所有数据必须经由裁判系统。
4. 摩擦轮预热 (`RunningTask=5`) 在切换至发射任务后自动退出。
5. `get_dart_param` API 是消费式接口 — 每次调用会消费一包数据，确保不重复发射。CDC 反馈中使用的是非消费式直读。">
