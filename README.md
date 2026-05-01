# Aeronautics 飞控系统 - 使用说明书

## 一、前置条件

### 必需的模组

| 模组 | 获取地址 | 说明 |
|---|---|---|
| **Create** | modrinth.com/mod/create | 机械动力本体 |
| **Create Aeronautics** | modrinth.com/mod/create-aeronautics | 航空学扩展 |
| **CC: Tweaked** | modrinth.com/mod/cc-tweaked | 计算机模组 |
| **CC: Sable** | modrinth.com/mod/cc-sable | **必须！** CC 与 Aeronautics 的桥梁 |

> 四个模组缺一不可，特别是 **CC: Sable**，没有它电脑无法读取飞行数据。

---

## 二、硬件搭建

### 1. 飞机结构总览

```
                    螺旋桨 (Propeller)
                         │
                    ┌────┴────┐
                    │  机头   │
                    └────┬────┘
           ┌─────────────┼─────────────┐
           │             │             │
     ┌─────┴─────┐ ┌─────┴─────┐ ┌─────┴─────┐
     │ 左翼副翼  │ │   机身    │ │ 右翼副翼  │
     │ Swivel +  │ │           │ │ Swivel +  │
     │ Torsion   │ │  CC电脑   │ │ Torsion   │
     │ Spring    │ │           │ │ Spring    │
     └───────────┘ └─────┬─────┘ └───────────┘
                         │
                    ┌────┴────┐
                    │  尾部   │
                    │升降舵+方向舵│
                    └─────────┘
```

### 2. CC 电脑外设连接表

| 电脑侧面 | 摆放方块 | 朝向 | 说明 |
|---|---|---|---|
| **bottom** | Gimbal Sensor (万向节传感器) | 任意 | 读取俯仰角 + 滚转角 |
| **top** | Altitude Sensor (高度计) | 朝上 | 读取高度 + 气压 |
| **back** | Velocity Sensor (速度计) | **朝向机头方向** | 读取空速 |
| **front** | Navigation Table (导航台) | 朝前 | 可选，用于航向控制 |
| **right** | Sequenced Gearshift | 齿轮朝向升降舵 | 控制升降舵 |
| **left** | Sequenced Gearshift | 齿轮朝向左副翼 | 控制左副翼 |
| **bottom** | Sequenced Gearshift | 齿轮朝向右副翼 | 控制右副翼 |
| **top** | Sequenced Gearshift | 齿轮朝向方向舵 | 控制方向舵（可选） |
| **back** | Rotation Speed Controller | 连接旋转轴 | 控制螺旋桨油门 |

> **重要！** 由于 Sable 的 Bug #638，外设贴在 top/bottom 面最安全，旋转后不会错位。
> 如果外设不够用，可以用红石线延长连接。

### 3. 舵面机械结构

每个舵面需要这样的传动链：

```
[Sequenced Gearshift] ──轴──► [Swivel Bearing 齿轮端]
                                         │
                                  [Swivel Bearing 主体]
                                         │
                                   ┌─────┴─────┐
                                   │ Torsion   │  ← 限制最大偏转角 + 回中
                                   │ Spring    │
                                   └─────┬─────┘
                                         │
                                   [舵面帆 Sail]
```

**舵面安装方向：**

| 舵面 | Swivel Bearing 轴朝向 | 作用 |
|---|---|---|
| 升降舵 (Elevator) | 水平朝向侧面 | 控制俯仰 (抬头/低头) |
| 左副翼 (Left Aileron) | 水平朝外 | 控制滚转 (左倾) |
| 右副翼 (Right Aileron) | 水平朝外 | 控制滚转 (右倾) |
| 方向舵 (Rudder) | 竖直朝上 | 控制偏航 (左转/右转) |

### 4. 动力结构

```
[Rotation Speed Controller] ──轴──► [齿轮组] ──► [螺旋桨 Propeller]
     ↑ CC 电脑连接
```

---

## 三、软件部署

### 1. 文件复制到 CC 电脑

将整个 `cc-aeronautics-flightctrl` 文件夹复制到 CC 电脑根目录：

```
CC 电脑文件结构:
/
├── config/
│   └── settings.lua
├── lib/
│   ├── init.lua
│   ├── pid.lua
│   └── hardware.lua
├── programs/
│   └── flightctrl.lua
└── API_REFERENCE.md
```

**复制方法：**
- 用 floppy disk 写入文件，插入电脑的 drive
- 或者用 modem + `file_transfer` 事件传输
- 或者在电脑上直接 `edit` 创建文件

### 2. 修改配置文件

编辑 `config/settings.lua`，根据你的实际连接修改 `CONFIG.peripherals`：

```lua
CONFIG.peripherals = {
    gimbal = "bottom",           -- 万向节传感器在哪一面
    altitude = "top",            -- 高度计在哪一面
    velocity = "back",           -- 速度计在哪一面
    navigation = "front",        -- 导航台（没有就填 nil）
    elevator = "right",          -- 升降舵 Sequenced Gearshift
    aileron_left = "left",       -- 左副翼 Sequenced Gearshift
    aileron_right = "bottom",    -- 右副翼 Sequenced Gearshift
    rudder = "top",              -- 方向舵（没有就填 nil）
    throttle = "back",           -- Rotation Speed Controller
    display = "front",           -- Display Link（没有就填 nil）
}
```

**没有的方块填 `nil`：**
```lua
navigation = nil,    -- 没有导航台
rudder = nil,        -- 没有方向舵
display = nil,       -- 没有显示屏
```

### 3. 调整 PID 参数（可选）

如果飞机飞行不稳定，调整 `config/settings.lua` 中的 PID 值：

```lua
CONFIG.pid = {
    throttle = { kp = 0.8, ki = 0.02, kd = 0.15 },  -- 油门 PID
    pitch    = { kp = 0.4, ki = 0.01, kd = 0.2 },   -- 俯仰 PID
    roll     = { kp = 0.6, ki = 0.015, kd = 0.25 }, -- 滚转 PID
    altitude = { kp = 0.15, ki = 0.005, kd = 0.05 }, -- 高度 PID
}
```

**调参指南：**

| 症状 | 调整方法 |
|---|---|
| 振荡/抖动 | 减小 `kp`，增大 `kd` |
| 响应太慢 | 增大 `kp` |
| 达不到目标值 | 增大 `ki` |
| 过度超调 | 减小 `kp`，增大 `kd` |

---

## 四、运行程序

### 启动

在 CC 电脑的终端输入：

```
lua programs/flightctrl.lua
```

看到启动画面：
```
╔══════════════════════════════════════════════╗
║   Aeronautics Flight Controller v1.0        ║
║   Type 'help' for commands                   ║
╚══════════════════════════════════════════════╝
[FC] System initialized. Sensors: gimbal, altitude, velocity
[FC] Awaiting pilot commands...
[FC] >
```

### 命令列表

| 命令 | 说明 | 示例 |
|---|---|---|
| `help` | 显示帮助 | `help` |
| `manual` | 手动键盘控制模式 | `manual` |
| `auto [速度] [高度]` | 自动驾驶（默认 128 RPM, 80 格） | `auto 128 80` |
| `hover [高度]` | 悬停模式 | `hover 60` |
| `land` | 自动降落 | `land` |
| `throttle <RPM>` | 直接设置油门 | `throttle 128` |
| `setalt <Y>` | 设置目标高度 | `setalt 100` |
| `setspeed <RPM>` | 设置目标速度 | `setspeed 150` |
| `status` | 查看传感器数据 | `status` |
| `test` | 测试所有舵面 | `test` |
| `emergency` | 紧急停止 | `emergency` |
| `stop` | 关闭系统 | `stop` |

### 手动模式按键

进入 `manual` 模式后：

| 按键 | 功能 |
|---|---|
| `↑` | 抬头（升降舵上偏） |
| `↓` | 低头（升降舵下偏） |
| `←` | 左滚转（左副翼下偏） |
| `→` | 右滚转（右副翼上偏） |
| `Q` | 增加油门 (+16 RPM) |
| `A` | 减少油门 (-16 RPM) |
| `S` | 停止引擎 |
| `W` | 左偏航（方向舵左偏） |
| `D` | 右偏航（方向舵右偏） |
| `E` | 所有舵面回中 |

---

## 五、飞行流程

### 起飞前检查

```
1. 运行程序: lua programs/flightctrl.lua
2. 输入: test          ← 测试所有舵面和油门
3. 观察舵面是否正确偏转
4. 输入: status        ← 确认传感器有读数
5. 确认 Gimbal Sensor 显示 pitch=0, roll=0（飞机水平）
```

### 手动起飞

```
1. 输入: manual
2. 按 Q 多次 → 油门推到 128+ RPM
3. 飞机加速后，按 ↑ 抬头起飞
4. 达到目标高度后按 E 回中舵面
```

### 切换到自动驾驶

在手动飞行过程中，命令行提示符仍然可用：

```
[FC] > auto 128 80     ← 切换到自动驾驶，保持 80 格高度
```

### 自动降落

```
[FC] > land            ← 开始自动降落程序
```

降落分三个阶段：减速 → 下降 → 停止

### 紧急情况

```
[FC] > emergency       ← 紧急停止：油门归零 + 舵面回中 + 机翼回平
```

---

## 六、常见问题

### Q: 程序报错 "attempt to call field 'getAngularVelocity' (a nil value)"

**A:** 电脑不在装配体上（还没有用 Physics Assembler 组装）。先组装飞机再运行程序。

### Q: 传感器读数为 0

**A:** 检查 `config/settings.lua` 中的外设方向是否与实际摆放一致。

### Q: 舵面不转动

**A:** 检查传动链：Sequenced Gearshift → 轴 → Swivel Bearing 齿轮端，确保轴连接正确。

### Q: 飞机左右摇摆不稳定

**A:** PID 参数需要调整。减小 `roll` 的 `kp`，增大 `kd`。参考第三章的调参指南。

### Q: 高空推力不够

**A:** 这是正常的，空气稀薄导致推力下降。适当增加油门目标值 `setspeed 180`。

### Q: 旋转飞机后外设方向乱了

**A:** Sable Bug #638。把外设贴在 top/bottom 面可以避免。如果已经乱了，重启程序后修改 `config/settings.lua` 中的方向。

### Q: 导航台 heading 一直显示 0

**A:** 导航台需要放入导航物品（罗盘/地图）。放入指向目标地点的 Lodestone Compass 或地图。

---

## 七、高级用法

### 自定义飞行脚本

飞控程序支持在飞行中通过命令行切换模式：

```lua
-- 巡航飞行
auto 128 80

-- 爬升到 120 格
setalt 120

-- 悬停
hover 120

-- 降落
land
```

### 与其他程序联动

可以通过 `rednet` 或 `modem` 接收外部指令控制飞机。飞控程序的核心 API 在 `lib/hardware.lua` 中：

```lua
local Hardware = require("lib/hardware")
local CONFIG = require("config/settings")
local hw = Hardware.new(CONFIG)
hw:initPeripherals()

-- 直接控制
hw:setThrottle(128)
hw:setElevator(10)
hw:setAilerons(-5)
hw:setRudder(3)
```
