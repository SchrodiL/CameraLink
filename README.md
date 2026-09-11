# CamLink

**多设备相机控制器** —— 一块 ESP32-C6，同时控制 DJI 与 Insta360 运动相机，并把 GPS 数据推给飞控 OSD。

> 本项目 fork 自 DJI 官方 demo [`dji-sdk/Osmo-GPS-Controller-Demo`](https://github.com/dji-sdk/Osmo-GPS-Controller-Demo)。
> 原项目是一个 DJI 专用的 Osmo Action 蓝牙快门遥控器；CamLink 把它重构成**协议无关的控制器核心 + 可插拔的协议后端**，
> DJI 只是其中一个后端。

---

## 它做什么

一块挂在遥控器（或飞控）上的小板子，同时扮演两个角色：

**对相机** —— 通过 BLE 冒充官方遥控器，直接控制相机：

| 功能 | DJI | Insta360 |
|---|---|---|
| 快门 / 录制开关 | ✅ | ✅ |
| 切换拍摄模式 | ✅ | ✅ |
| 切换相机端预设 | ✅ | ✅ |
| 睡眠 / 唤醒 | ✅ | ✅ |
| 关机 | ❌ 协议无此命令 | ✅ |
| 深度唤醒（相机已关机） | ✅ 广播 | ✅ 唤醒信标 |
| 电量 / 剩余容量 | ✅ 精确百分比 | ✅ 5 档位 + 充电状态 |
| 拍摄规格 | ✅ | ✅ |

**对飞控** —— 通过 MSP 往 Betaflight/INAV 的 OSD 推 4 条自定义文本（相机状态、电量、GPS 等），
并读取 16 个 RC 通道来做功能映射。

---

## 硬件

- **主控**：ESP32-C6（RISC-V，ESP-IDF v6.0）
- **GPS**：LC76G GNSS 模块（UBX/NMEA，接 UART）
- **飞控**：任意支持 MSP 的 Betaflight/INAV 飞控（接 UART）
- **按键**：BOOT 键（单击拍录 / 长按对频 / 双击切协议）
- **指示灯**：板载 RGB LED

> 引脚定义见 `core/hardware_config.h`。

---

## 项目结构

```
core/                协议无关的控制器核心
  camera_backend.*     后端接口（vtable）+ 活动后端注册/切换
  controller.*         统一命令门面 + 单一控制工作队列
  camera_state.*       统一相机状态缓存
  pairing.*            对频流程（委托给活动后端）
  ble_common.*         BLE 栈统一拉起

adapters/            协议后端 —— 新增一种设备 = 新写一个目录
  dji/                 DJI R SDK 协议（GATTC 主机角色）
  insta360/            Insta360 遥控协议（GATTS 从机角色）

services/            协议无关功能
  gps / osd / osd_config / channel_map / key / light / web_server

shared/              通用工具（枚举、NVS、CRC）
components/msp/      可复用的 MSP 协议组件
main/                应用装配 + 内嵌 WebUI
```

**新增一种相机**只需要在 `adapters/` 下实现 `camera_backend_t` 那几个函数，核心逻辑不用动。

---

## 主要功能

### 相机控制

两种协议的角色正好相反，这是本项目最需要留意的差异：

- **DJI** —— 模块是 GATT **主机**，主动扫描并连接相机（相机广播，我们连它）
- **Insta360** —— 模块是 GATT **从机**，冒充官方遥控广播，等相机连进来

> ⚠️ Insta360 侧**广播名必须是 `Insta360 GPS Remote`**。实测把名字改成别的相机就完全扫不到模块
> （表现为「模块绿灯亮、相机连不上」）。相机是按名字精确匹配设备的，不是靠服务 UUID。

### OSD

通过 `MSP2_SET_TEXT` 往飞控发 4 条自定义文本，内容从内容池里任选：

录制状态 · 相机电量 · 录制规格 · 相机名称+剩余容量 · GPS 卫星数 · 速度+海拔 · GPS 纬度 · GPS 经度

单条上限 16 字符，所以经纬度拆成了两条。

### 通道映射

Betaflight Modes 风格：每个功能选一个 RC 通道，再在行程条上拖出触发范围。

- 行程条上两个蓝色手柄之间 = 触发范围，可拖手柄改两端、**拖中间整条平移**
- 细竖线 = 通道当前位置，落进范围即高亮整个功能区
- 下拉里选「自动检测…」后拨动开关，自动认出是哪个通道

### WebUI

板子自己开一个 WiFi 热点提供网页配置界面：状态、OSD 布局、协议切换、对频、WiFi 空闲、
通道映射、固件刷写、配置备份/恢复、恢复默认设置。

### 安全设计

- **NVS 里绝不在 BLE 回调中写入** —— flash 写会阻塞蓝牙协议栈导致连接掉线，所有落盘都推迟到任务上下文
- **控制动作统一走一个工作队列** —— 按键扫描和 OSD 任务绝不阻塞在 BLE 命令上
- **配置带版本号** —— 功能枚举增删时换 NVS 键名，避免旧配置被按新索引**错位解读**

---

## 构建

需要 **ESP-IDF v6.0**，目标芯片 `esp32c6`：

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p <PORT> flash monitor
```

首次构建会拉取 `managed_components/` 下的依赖。

---

## 协议文档

- **DJI R SDK** —— 见 `docs/`，来自上游官方文档
- **Insta360 配件协议** —— 本项目通过 nRF52840 空口抓包逆向，独立仓库：
  **[insta360-ble-protocol](https://github.com/SchrodiL/insta360-ble-protocol)**

---

## 第三方资源

本项目网页内嵌了两个第三方字体（仅用于标题）：

- **ROG Fonts** —— ASUS，官方免费公开提供
- **造字工坊凌黑体（子集）** —— 文件名标注 Noncommercial，**非商用授权**

如果你是商业用途，请自行移除或替换（见 `main/web/index.html` 里的 `@font-face`）。

---

## 许可

MIT。上游 DJI demo 的原始版权声明保留在各自源文件头部。

---

## English

**CamLink** turns an ESP32-C6 into a multi-device camera controller: it speaks both the DJI R SDK
protocol (as a GATT client connecting *to* the camera) and the Insta360 remote protocol (as a GATT
server the camera connects *to*), and pushes GPS/telemetry to a Betaflight OSD over MSP.

Forked from DJI's official `Osmo-GPS-Controller-Demo` and restructured into a protocol-agnostic
core plus pluggable backends — adding a new camera family means adding one directory under
`adapters/`.

Key design notes: no NVS writes inside BLE callbacks (blocks the stack and drops the link); all
control actions go through a single work queue so key scanning and OSD never block on BLE; config
blobs are version-keyed so enum changes can't silently misinterpret saved settings.

The Insta360 protocol was reverse-engineered from on-air captures — see
[insta360-ble-protocol](https://github.com/SchrodiL/insta360-ble-protocol).
