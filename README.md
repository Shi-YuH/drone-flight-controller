# 四轴无人机飞控系统 · 基于 FreeRTOS 的多任务嵌入式系统

双固件项目：**飞行器（飞控）** + **遥控器**，各自运行在独立的 STM32F103 上，通过 2.4G 无线通信。
完成了姿态解算、串级 PID 控制、定高、无线遥控、状态机与故障保护，并完成实机试飞与参数整定。

---

## 1. 项目概览

| 项目 | 内容 |
|---|---|
| 主控 | STM32F103（遥控器与飞控板各一片） |
| 开发方式 | **STM32CubeMX + HAL 库 + VSCode/GCC**（与另一项目的标准库裸机开发形成互补） |
| 实时系统 | **FreeRTOS**（任务创建、周期调度、直接任务通知） |
| 姿态传感器 | MPU6050 六轴 IMU（I2C，驱动移植自厂商资料） |
| 无线通信 | SI24R1 2.4G 收发芯片（SPI，驱动移植自厂商资料） |
| 测距（定高） | VL53L1X 激光测距（ToF，I2C） |
| 显示 | OLED（I2C） |
| 电源管理 | IP5305T（升降压 + 按键开关机） |
| 输入 | 双摇杆（ADC + DMA）、按键 |
| 控制周期 | 10ms（姿态解算 + PID） |

---

## 2. 软件架构

两个固件采用相同的三层结构，层间单向依赖：

```
┌──────────────────────────────────────────────────────┐
│  Application（应用层 / 业务逻辑）                      │
│  飞控板：App_flight         姿态解算 + PID + 电机混控   │
│         App_receieve_data  无线数据接收与解析          │
│         App_freeRTOS_Task  任务划分与状态机            │
│  遥控器：App_process_data  摇杆/按键数据处理            │
│         App_transmit_data  遥控数据组帧发送            │
│         App_display        OLED 显示                   │
│         App_freeRTOS_Task  任务划分                    │
├──────────────────────────────────────────────────────┤
│  common（通用算法层 / 可复用）                         │
│  Com_imu      四元数姿态解算（Mahony 互补滤波）        │
│  Com_filter   一阶低通滤波 + 卡尔曼滤波                │
│  Com_pid      PID 控制器（单环 / 串级）                │
│  Com_config   全局配置    Com_debug  日志输出          │
├──────────────────────────────────────────────────────┤
│  interface（硬件接口层 / 驱动）                        │
│  int_mpu6050  int_SI24R1  int_motor  int_led          │
│  int_bat_adc  int_P5305T  int_oled  int_font          │
│  int_joystick int_key     fix_height/int_VL53L1       │
├──────────────────────────────────────────────────────┤
│  STM32 HAL 驱动 + CMSIS + FreeRTOS（CubeMX 生成）      │
└──────────────────────────────────────────────────────┘
```

**分层原则**：`Application` 只调用 `common` 与 `interface`；`common` 不依赖任何具体硬件，
可整体移植到其他平台；`interface` 封装全部寄存器/句柄操作，上层不直接碰 HAL。

**共用设计**：`int_SI24R1`、`int_oled`、`int_font` 在飞控板与遥控器两个工程中**完全复用**，
只改引脚宏定义。

---

## 3. FreeRTOS 任务设计

### 3.1 飞控板任务

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| `Com_Task` 通信 | **4** | 10ms | 接收遥控数据、处理链接状态、关机命令、电池电压回传 |
| `Power_Task` 电源 | **4** | 10s | 周期触发 IP5305T 防止自动关机；收到通知则执行关机 |
| `Flight_Control_Task` 飞控 | **3** | 10ms | 姿态解算 → PID 计算 → 电机控制 |
| `LED_Task` 指示 | **1** | 100ms | 按连接状态与飞行状态驱动四路 LED |

**优先级设计思路**：通信与电源涉及安全（失控、掉电），优先级最高；
飞控需要确定性周期，次之；LED 纯指示，最低。

### 3.2 任务间同步：直接任务通知

没有使用信号量或队列，而是用 **FreeRTOS 直接任务通知（`xTaskNotifyGive` / `ulTaskNotifyTake`）**
在任务间传递事件，开销更小、无需额外的内核对象：

```
故障缓停完成 → xTaskNotifyGive(com_task_hander)   → 唤醒通信任务继续处理状态
收到关机命令 → xTaskNotifyGive(power_task_hander) → 唤醒电源任务执行关机
```

---

## 4. 姿态解算（Mahony 互补滤波 + 四元数）

这是本项目的算法核心，实现在 `common/Com_imu.c`。

### 4.1 处理流程

```
MPU6050 六轴原始数据
        │
        ├─ 陀螺仪 ──→ 一阶低通滤波（α = 0.15，去高频噪声）
        │
        └─ 加速度 ──→ 卡尔曼滤波（三轴各一个滤波器）
        │
        ↓
Mahony 互补滤波 + 四元数更新
        ↓
     欧拉角（pitch / roll / yaw）
```

### 4.2 Mahony 算法的关键步骤

**① 用当前四元数推算重力方向**

```c
Gravity.x = 2 * (q1*q3 - q0*q2);
Gravity.y = 2 * (q0*q1 + q2*q3);
Gravity.z = 1 - 2 * (q1*q1 + q2*q2);
```

**② 加速度归一化** —— 使用**快速倒数平方根**（`Q_rsqrt`，含一次牛顿迭代），
避免调用 `sqrt` 的开销：

```c
i = 0x5f3759df - (i >> 1);          // 魔数初值
y = y * (1.5f - x2 * y * y);        // 一次牛顿迭代
```

**③ 叉积求误差，并用 PI 修正陀螺仪零偏**

```c
AccGravity = Acc × Gravity;                 // 测量重力与估计重力的偏差
GyroIntegError += AccGravity * Ki;          // 积分项消除稳态零偏
Gyro = gyro_raw * Gyro_Gr + Kp * AccGravity + GyroIntegError;
```

参数：**Kp = 0.8，Ki = 0.0003**。Kp 决定加速度计对姿态的修正强度，
Ki 用于消除陀螺仪的累积零偏。

**④ 一阶龙格库塔法更新四元数，然后归一化**

**⑤ 解算欧拉角**

```c
pitch = asin(vecxZ) * RtA;                  // 俯仰角
roll  = atan2f(vecyZ, veczZ) * RtA;         // 横滚角
yaw  += gyro_z * dt;                        // 偏航角只能靠角速度积分
```

**⑥ 偏航角死区处理** —— 角速度小于 ±0.5°/s 时视为干扰，不参与积分，
避免静止时偏航角缓慢漂移：

```c
if ((yaw_G > 0.5f) || (yaw_G < -0.5f)) eulerAngle->yaw += yaw_G * dt;
```

### 4.3 为什么用四元数而不是直接互补滤波

项目里保留了一段被注释掉的**直接互补滤波**实现（用 `atan2` 从加速度反推角度）：

```c
// euler_data.pitch = atan2(accel_x, accel_z) / 3.14159 * 180;   // 未采用
```

**未采用的原因**：直接反推在**大角度机动**时误差大，且无法处理偏航轴的耦合。
四元数法用完整的旋转矩阵描述姿态，**没有万向节死锁问题**，更适合飞控这种连续机动的场景。

---

## 5. 串级 PID 控制

### 5.1 双闭环结构

三轴各自独立的一对串级 PID：

```
目标角度 ──→ [外环 角度环] ──→ 目标角速度 ──→ [内环 角速度环] ──→ 混控 ──→ 电机
                  ↑                                    ↑
            欧拉角（解算输出）                    陀螺仪角速度
```

**内环为什么必须存在**：角度环输出"期望的转动快慢"，角速度环负责快速跟随。
只有角度环会导致响应迟滞、容易振荡；加上内环相当于给系统加了阻尼，稳定性显著提升。

### 5.2 参数（`App_flight.c`）

| 轴 | 外环（角度） | 内环（角速度） |
|---|---|---|
| 俯仰 Pitch | Kp = −7.00 | Kp = 3.00，Kd = 0.50 |
| 横滚 Roll | Kp = −7.00 | Kp = 3.00，Kd = 0.50 |
| 偏航 Yaw | Kp = −5.00 | Kp = −5.00 |
| 定高 | Kp = −0.60，Kd = −0.20 | — |

**符号为什么是负的**：误差定义为 `err = measure - desired`（测量值减目标值），
所以增益取负号才能形成负反馈。这个符号约定在 `Com_pid.c` 里统一：

```c
pid->err = pid->measure - pid->desired;
pid->output = kp*err + (ki*integral)*PID_PERIOD + (kd*diff)/PID_PERIOD;
```

**积分项和微分项为什么乘/除周期**：把离散求和的积分、离散差分的微分换算成
与连续时间 PID 一致的量纲，这样**改控制周期时不用重新整定参数**。

### 5.3 串级计算实现

```c
void Common_PID_Calc_Series(PID_Struct *out_pid, PID_Struct *in_pid)
{
    Common_PID_Calc(out_pid);              // 先算外环
    in_pid->desired = out_pid->output;     // 外环输出作为内环目标
    Common_PID_Calc(in_pid);               // 再算内环
}
```

---

## 6. 电机混控（X 型四轴）

四路电机挂在**四个不同的定时器**上：

| 电机 | 定时器 / 通道 |
|---|---|
| 左上 | TIM3_CH1 |
| 右上 | TIM2_CH2 |
| 左下 | TIM4_CH4 |
| 右下 | TIM1_CH3 |

混控公式（`App_flight.c`）：

```c
motor_top_left.speed     = thr + y_pid - x_pid + z_pid;
motor_top_right.speed    = thr + y_pid + x_pid - z_pid;
motor_bottom_left.speed  = thr - y_pid - x_pid - z_pid;
motor_bottom_right.speed = thr - y_pid + x_pid + z_pid;
```

**读法**：
- `thr` 是油门基准（四路共用）
- `y_pid`（俯仰）让前后两组**差动** → 产生俯仰力矩
- `x_pid`（横滚）让左右两组**差动** → 产生横滚力矩
- `z_pid`（偏航）让**对角两组同向、另一对角反向** → 利用反扭矩差产生偏航力矩

### 6.1 安全保护

```c
// 限幅：电机速度限制在 0~700
speed = Com_limit(speed, 0, 700);

// 低油门强制停转：油门 < 50 时直接置 0，避免地面误转
if (remote_data.thr < 50) { 四路电机 speed = 0; }
```

---

## 7. 定高功能

使用 **VL53L1X ToF 激光测距**测量对地高度，做**单环 PID**：

```c
height_pid.desired = fix_height;                    // 按下定高时的锁定高度
height_pid.measure = int_VL53L1_get_distance();     // 当前实测高度
Common_PID_Calc(&height_pid);
```

定高状态下的电机速度为：`thr + 三轴 PID 输出 + height_pid.output`（高度输出四路同加）。

> **注意**：VL53L1X 的官方 API（`VL53L1X_api.c`）与校准库来自 ST 官方，
> 本项目的 `vl53l1_platform.c` 是**平台移植层**（把 ST 的 API 对接到底层 I2C 读写），
> `int_VL53L1.c` 是**业务封装层**。

---

## 8. 飞行状态机与故障保护

```
        ┌────────┐  遥控连接 + 解锁   ┌────────────┐
        │  IDLE  │ ─────────────────→ │   NORMAL   │
        │  空闲  │ ←───────────────── │  正常飞行  │
        └────────┘      上锁/故障      └────────────┘
             ↑                              ↓ 按定高
             │                        ┌────────────┐
             │                        │ FIX_HEIGHT │
             │                        │   定高     │
             │                        └────────────┘
             │ 缓停完成（任务通知）
        ┌────┴────┐
        │  FAIL   │  故障态：电机每周期降 3，降到 0 后通知通信任务
        └─────────┘
```

**故障处理的实现**（`App_flight.c`）：

```c
case FLIGHT_STATE_FAIL:
    // 每 6ms 电机速度降 3，缓慢停机而不是骤停（避免空中失速坠落）
    motor_top_left.speed -= 3;  /* ... 四路同理 ... */
    if (全部 <= 0) xTaskNotifyGive(com_task_hander);   // 通知通信任务收尾
    break;
```

**LED 状态指示**（`App_freeRTOS_Task.c`）：

| 状态 | 前两灯（连接） | 后两灯（飞行状态） |
|---|---|---|
| 未连接 / 连接 | 灭 / 亮 | — |
| IDLE | — | 慢闪（500ms） |
| NORMAL | — | 快闪（200ms） |
| FIX_HEIGHT | — | 常亮 |
| FAIL | — | 全灭 |

---

## 9. 无线通信（SI24R1）

- **SPI 驱动**：将厂商参考代码移植到 STM32 HAL 平台（`HAL_SPI_TransmitReceive` 替换底层读写），
  实现 SI24R1 的寄存器读写、工作模式切换与收发通道配置
- **双端任务**：遥控器 `App_transmit_data.c` 组帧发送；飞控板 `App_receieve_data.c` 接收解析
- **帧格式**：含帧头与校验，解析异常时丢弃并维持上一状态，避免误动作
- **链接状态检测**：连续未收到有效数据则判定断连，进入 `REMOTE_STATE_DISCONNECTED`
- **电池电压回传**：飞控通过 ADC 测量电池电压，格式化为字符串后经无线回传遥控器显示

---

## 10. 工程问题记录

> 以下为实际调试中遇到的问题与定位过程。

### 10.1 电源芯片误判按键（两行代码顺序导致）

**现象**：刚开机时，短时间按第二下电源键会失灵，或者直接被判定为关机。

**定位过程**：IP5305T 的开关机逻辑是**按一次开机、按两下关机**。
排查发现刚开机时程序会立即执行一次"周期重启"指令——
而**这个重启动作本身就是模拟一次按键**，于是被芯片当成了一次真实按键。
用户紧接着再按一下，就凑成了"两下"，触发关机。

**根因**：`vTaskDelayUntil` 与 `Int_IP5305T_start()` 的**执行顺序颠倒**——
先执行了重启，才进入延时。

```c
// 错误顺序：先重启再延时 → 开机瞬间就模拟了一次按键
Int_IP5305T_start();
vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));

// 正确顺序：先延时，让开机按键动作过去，再进入周期重启
vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));
Int_IP5305T_start();
```

**解决**：调整为"先延时 10 秒再执行重启"，问题消失。

**经验**：这类"硬件芯片的输入被软件误触发"的问题，
排查时要把**软件动作与硬件行为逐一对应**——尤其是共用同一个物理按键的场景。

### 10.2 电源芯片自动关机

**现象**：无人机工作一段时间后自行关机。

**原因**：IP5305T 自带低功耗逻辑，**一定时间无操作会自动关机**。

**解决**：新增 `Power_Task`，每 10s 通过 GPIO 模拟一次按键动作，
"喂"住电源芯片维持供电；同时该任务还承担接收关机通知的职责
（用直接任务通知与通信任务配合）。

### 10.3 偏航角静止漂移

**现象**：飞行器静止时，偏航角读数缓慢增长。

**原因**：偏航角只能靠陀螺仪 Z 轴角速度积分得到，
而陀螺仪存在零偏，静止时的微小噪声被持续积分。

**解决**：加入**死区判断**——角速度绝对值小于 0.5°/s 时不参与积分，
视为干扰而非真实转动。

### 10.4 PID 参数整定过程

**整定顺序**：**先内环（角速度环），后外环（角度环）**。

1. 先只调内环 Kp，观察角速度响应是否快速跟随目标
2. 内环稳定后加 Kd 抑制超调
3. 再调外环 Kp，观察角度能否回到目标值
4. 最后叠加偏航轴与定高环

**遇到的现象**：初期直立环 Kp 偏小时回正无力、机体缓慢倾倒；
Kp 加大后又出现高频抖动，需要靠内环 Kd 抑制。

> **本项目未做的一件事**：`Com_pid.c` 的积分项**没有抗饱和（anti-windup）处理**。
> 当前参数下 `ki` 均为 0，所以未暴露问题；若后续要启用积分项，
> 需要补充积分限幅或积分分离，避免执行器饱和后积分持续累积导致超调。

---

## 11. 目录结构

仓库根目录下为两套独立固件工程：

```
├── Part_01_aircraft_hal/          飞控板固件
│   ├── MDK-ARM/
│   │   ├── Application/           应用层：飞控算法、数据接收、任务划分
│   │   │   ├── App_flight.c             姿态解算 + PID + 电机混控
│   │   │   ├── App_receieve_data.c      无线数据接收与解析
│   │   │   └── App_freeRTOS_Task.c      任务创建、状态机、状态指示
│   │   ├── common/                通用算法层（不依赖具体硬件，可移植）
│   │   │   ├── Com_imu.c                四元数姿态解算（Mahony）
│   │   │   ├── Com_filter.c             一阶低通 + 卡尔曼滤波
│   │   │   ├── Com_pid.c                PID（单环 / 串级）
│   │   │   ├── Com_config.c             全局配置
│   │   │   └── Com_debug.c              日志输出
│   │   ├── interface/             硬件接口层
│   │   │   ├── int_mpu6050.c            IMU 驱动（I2C）
│   │   │   ├── int_SI24R1.c             2.4G 无线驱动（SPI）★与遥控器共用
│   │   │   ├── int_motor.c              电机 PWM 输出
│   │   │   ├── int_led.c  int_bat_adc.c  int_P5305T.c
│   │   │   ├── oled/                    OLED 与字库 ★与遥控器共用
│   │   │   └── fix_height/              VL53L1X 激光测距（含 ST 官方 API）
│   │   └── freeRTOS/              FreeRTOS 内核
│   ├── Core/                      CubeMX 生成的主程序、中断与初始化
│   ├── Drivers/                   STM32 HAL 驱动 + CMSIS
│   └── Part_01_aircraft_hal.ioc   CubeMX 工程配置
│
└── Part_02_remote_hal/            遥控器固件
    ├── MDK-ARM/
    │   ├── Application/
    │   │   ├── App_process_data.c       摇杆/按键数据处理
    │   │   ├── App_transmit_data.c      遥控数据组帧发送
    │   │   ├── App_display.c            OLED 显示
    │   │   └── App_freeRTOS_Task.c      任务划分
    │   ├── common/  interface/          同飞控板结构
    │   └── freeRTOS/
    ├── Core/  Drivers/
    └── Part_02_remote_hal.ioc
```

**命名说明**：`Part_01` 为**飞控板**（aircraft）、`Part_02` 为**遥控器**（remote），
目录名沿用工程的原始命名，以保证 **Keil 工程内的相对路径可直接使用、无需重新配置**。

**代码来源划分**：

| 目录 / 文件 | 来源 | 说明 |
|---|---|---|
| `MDK-ARM/Application/` | **本人编写** | 飞控算法、任务划分、状态机、数据处理 |
| `MDK-ARM/common/` | **本人编写** | 姿态解算、滤波、PID、配置、日志 |
| `MDK-ARM/interface/` 下的 `int_*.c` | **移植适配** | 由芯片厂商资料/参考代码移植到 STM32 HAL 平台，**重写底层读写接口**，见下方说明 |
| `interface/fix_height/VL53L1X_*.c`、`vl53l1_*` | **ST 官方** | VL53L1X 的 API 与校准库；本项目的 `vl53l1_platform.c` 为平台移植层 |
| `Drivers/`（HAL + CMSIS） | **ST 官方** | 由 CubeMX 引入 |
| `MDK-ARM/freeRTOS/` | **FreeRTOS 官方** | 内核源码，由 CubeMX 引入 |

### 关于驱动层的工作边界

`interface/` 下的设备驱动（MPU6050、SI24R1、OLED 等）**不是从零编写**，
而是**由芯片厂商提供的资料/参考代码移植到本项目的硬件平台**。本人在其中的工作包括：

1. **重写底层读写接口**——厂商代码通常针对特定平台，需要改用 HAL 库接口：
   - MPU6050：`HAL_I2C_Mem_Write` / `HAL_I2C_Mem_Read`
   - SI24R1：`HAL_SPI_TransmitReceive`
   - OLED：I2C 时序与初始化序列适配
2. **打通初始化流程**——寄存器配置参数按本项目硬件调整
3. **补充厂商代码没有的业务逻辑**——例如 `Int_MPU6050_Calibration_offset()`
   中的**陀螺仪零偏校准与静置检测**（连续 100 次判断前后加速度差值小于阈值才认为停放平稳），
   属于本项目自行实现

> **诚实说明**：把"移植 + 适配 + 补齐业务逻辑"说成"自研驱动"是不准确的。
> 驱动层的寄存器操作逻辑来自官方资料，本人的贡献在**平台适配与上层封装**。

---

## 12. 如何编译

1. 安装 **Keil MDK** 与 **STM32F1xx 器件包**
2. 用 Keil 打开 `Part_01_aircraft_hal/MDK-ARM/` 下的工程文件编译烧录（飞控板）
3. 同样方式编译 `Part_02_remote_hal/MDK-ARM/`（遥控器）
4. CubeMX 工程文件（`.ioc`）可用于重新生成外设配置

**调试输出**：`Com_debug.c` 封装了 `debug_printf`，
通过串口输出姿态角、PID 输出、测距值等，便于结合上位机观察曲线。

---

## 13. 技术要点小结

- **FreeRTOS 多任务设计**：按实时性分配优先级，周期任务 + 直接任务通知实现同步
- **Mahony 互补滤波 + 四元数姿态解算**：含快速倒数平方根、一阶龙格库塔更新、偏航死区
- **串级 PID**：外环角度 + 内环角速度，误差符号与量纲处理统一在通用 PID 模块内
- **四路电机混控**：三轴 PID 输出按 X 型布局分配到四个独立定时器通道
- **多传感器接入**：完成 MPU6050（I2C）、SI24R1（SPI）、VL53L1X（ToF）、OLED 的
  驱动移植与平台适配，并补充零偏校准等业务逻辑
- **双固件协同**：遥控器与飞控板通过 2.4G 双向通信，含帧校验与链接状态检测
- **状态机与故障保护**：四态流转、故障缓停、低油门强制停转
- **软硬件协同排查**：从"按键误判"现象定位到代码执行顺序问题
