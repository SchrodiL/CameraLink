# Osmo Action GPS 蓝牙遥控器 (ESP32-C6-Example)

> 📌 **这是上游 DJI demo 的原始说明，保留供参考**（`docs/` 里有文档引用它的硬件连线图等章节）。
> 本仓库现在的项目说明请看 **[README.md](README.md)**。

  ![](https://img.shields.io/badge/version-V1.0.0-red.svg) ![](https://img.shields.io/badge/platform-rtos-blue.svg) ![](https://img.shields.io/badge/license-MIT-purple.svg)

<p align="center">
  <br><a href="README.md">English</a> | 中文
</p>

## 简介

此 Demo 提供了一套运行在 ESP32-C6 开发板上的代码（基于 ESP-IDF 框架），演示了如何解析、处理并发送 DJI R SDK 协议以控制相机。示例程序实现了基本的遥控功能，包括：长按 BOOT 按键连接到最近的（兼容的）Osmo Action、Osmo 360 设备、单击控制拍摄、快速切换模式以及基于 LC76G GNSS 模块进行 GPS 数据推送。同时，程序根据设备状态动态调整 RGB LED 灯的显示。

在阅读本文档与代码之前，建议先查看 [快速接入指南](docs/getting_started_guide_CN.md)。

## 主要功能特性

- **协议解析**：`protocol` 协议层展示了如何解析 DJI R SDK 协议，与**平台无关，便于移植**。
- **GPS 数据推送**：通过 LC76G GNSS 模块以 10Hz 频率收集 GPS 数据，经过解析后实时推送至相机。
- **按键支持**：支持单击（开始 / 停止录像）和长按（寻找并连接最近的相机）操作。
- **RGB 灯支持**：实时监控系统状态，根据状态变化动态调整 RGB LED 灯的颜色。
- **其他功能展示**：切换相机至指定模式、QS 快速切换模式、相机状态订阅、查询相机版本号等功能。

## 开发环境

**软件**：ESP-IDF v6.0

**硬件**：

- ESP32-C6-WROOM-1
- LC76G GNSS Module
- DJI Osmo 360 或 DJI Osmo Action 6 / 5 Pro / 4

硬件连接涉及 ESP32-C6-WROOM-1 与 LC76G GNSS Module 之间的连接，具体连接方式如下：

- **ESP32-C6 GPIO5** 连接到 **LC76G RX**
- **ESP32-C6 GPIO4** 连接到 **LC76G TX**
- **ESP32-C6 5V** 连接到 **LC76G VCC**
- **ESP32-C6 GND** 连接到 **LC76G GND**

请确保正确连接各个引脚，尤其是 TX 和 RX 引脚的互连，以保证数据传输的正常进行。

<img title="Hardware Wiring Diagram" src="docs/images/hardware_wiring_diagram.png" alt="Hardware Wiring Diagram" data-align="center" width="711">

## 快速开始

- 安装 ESP-IDF 工具链，具体安装步骤可参考下方的参考文档。我们推荐在 VSCode 中安装 ESP-IDF 插件，插件下载地址：[ESP-IDF 插件 - VSCode](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)

- 接下来，检查 Demo 中的 `.vscode/settings.json` 文件，确保 IDF 相关的参数（port、flashType）配置正确。

- 完成环境配置后，编译并烧录代码到开发板，使用 monitor 查看实时日志。您可以通过观察开发板上的 RGB 灯状态来了解当前设备状态：红色表示未初始化，黄色表示 BLE 初始化完成，设备已准备就绪。

- 长按 BOOT 按键时，RGB 灯蓝色闪烁，表示正在寻找并连接最近的 Osmo Action 设备。蓝色常亮表示 BLE 已连接，绿色常亮表示协议已连接，能够正常收发命令。紫色常亮表示协议已连接并且有 GPS 信号。

- 单击 BOOT 按键时，相机开始或结束拍摄。长时间录制时，RGB 灯会闪烁。

## Demo 结构

```
├── main/             # 应用装配（app_main + CMakeLists + web 前端）
├── core/             # 协议无关控制器核心
│                     #   camera_state / camera_backend / controller / pairing / ble_common
├── adapters/         # 相机协议后端（实现统一的 camera_backend_t 接口）
│   ├── dji/          # DJI 后端（GATTC 主机：BLE / 数据 / 协议帧 / 命令 / 连接 / 状态）
│   └── insta360/     # insta360 后端（GATTS 从机）
├── services/         # 协议无关功能（GPS / OSD / 通道映射 / profile / 按键 / 灯光 / WebUI）
├── shared/           # 共享工具与相机域词汇（NVS 辅助 / 相机枚举 / CRC）
├── components/msp/   # MSP 飞控 OSD 组件
└── CMakeLists.txt    # Demo 构建文件
```

- **core**：协议无关的核心——统一相机状态缓存（`camera_state`）、相机后端接口（`camera_backend`）、统一控制工作队列（`controller`）、对频编排（`pairing`）、BLE 栈统一拉起（`ble_common`）。
- **adapters**：相机协议后端，各实现一套 `camera_backend_t`；新增一种设备 = 新增一个 backend。
- **services**：协议无关的控制器功能（GPS 解析、MSP OSD、RC 通道映射、profile、按键、灯光、WebUI）。
- **shared**：通用工具与共享的相机域枚举（模式/状态/分辨率/帧率/防抖）。
- **main**：程序入口与应用装配。

## 程序启动时序图

<img title="Program Startup Sequence Diagram" src="docs/images/sequence_diagram_of_program_startup.png" alt="Program Startup Sequence Diagram" data-align="center" width="761">

## 协议解析说明

下面这张图展示了程序在解析帧时的大致流程：

<img title="Protocol Parsing Sequence Diagram" src="docs/images/sequence_diagram_of_protocol_parsing.png" alt="Protocol Parsing Sequence Diagram" data-align="center" width="500">

详细文档请参阅：[协议解析说明文档](docs/protocol_CN.md)

## GPS 推送示例

LC76G GNSS 模块的 GNRMC 和 GNGGA 数据最大支持 10Hz 的更新频率，前提是我们需要向该模块发送相应的指令：

```c
// "$PAIR050,1000*12\r\n" 为 1Hz 更新率
// "$PAIR050,500*26\r\n" 为 5Hz 更新率
// "$PAIR050,100*22\r\n" 为 10Hz 更新率
char* gps_command = "$PAIR050,100*22\r\n";  //（>1Hz 仅 RMC 和 GGA 支持）
uart_write_bytes(UART_GPS_PORT, gps_command, strlen(gps_command));
```

接下来启动一个任务，设置其优先级为 0。需要注意与其他任务的优先级配置，因为 GPS 推送频繁执行，可能会抢占其他任务的时间片。获取到的 GPS 原始数据如下：

```
$GNRMC,074700.000,A,2234.732734,N,11356.317512,E,1.67,285.57,150125,,,A,V*03
$GNGGA,074700.000,2234.732734,N,11356.317512,E,1,7,1.31,47.379,M,-2.657,M,,*65
```

在解析大量类似的字符串以提取经纬度、速度分量等信息时，需要剔除无效数据。为了减少由于漂移、定位误差等因素导致的不准确问题，建议在发送数据之前对GPS数据进行滤波和必要的处理。本程序目前并未深入考虑这些情况，未来可以根据需求引入合适的滤波算法和误差修正机制，以确保数据的准确性和可靠性。

由于解析过程频繁执行，任务执行时还需注意看门狗可能超时，因此程序中适当地使用了 `vTaskDelay` 来重置看门狗。本程序仅采用简单的解析方法，进行数据推送演示，请参阅 `gps_logic` 中的 `Parse_NMEA_Buffer` 和 `gps_push_data` 函数。

有 GPS 信号时（RGB 灯紫色常亮），开始录制一段视频，结束录制后可以在 DJI Mimo APP 的仪表盘中查看相应的数据。

## 如何添加功能

**在准备添加功能前，请确保已经完整阅读了 [协议解析说明文档](docs/protocol_CN.md) 和 [数据层说明文档](docs/data_layer_CN.md)。**

### 新增命令支持

发送或解析命令帧和应答帧时，只需简单的三步：

* 在 `dji_protocol_data_structures` 中定义帧结构体。

* 在 `dji_protocol_data_descriptors` 中定义三元组，并为其提供对应的 `creator` 和 `parser`。如果没有实现，可以设置为 `NULL`，当解析函数找不到对应的 `creator` 或 `parser` 时，构建或解析帧的过程将停止。

* 在逻辑层（`logic`）中定义相应函数，编写业务逻辑，调用命令逻辑（`command_logic`）中的 `send_command` 函数。

如果在逻辑层新增了 `.c` 文件，请确保修改 `main/CMakeLists.txt` 文件。

关于 `send_command` 函数，你需要知道：除了传入 `CmdSet`、`CmdID` 和帧结构体，还需要传入 `CmdType`，即帧类型，定义在 `enums_logic` 中：

```c
typedef enum {
    CMD_NO_RESPONSE = 0x00,      // 命令帧 - 发送数据后不需要应答                        
    CMD_RESPONSE_OR_NOT = 0x01,  // 命令帧 - 发送数据后需要应答，没收到结果不报错
    CMD_WAIT_RESULT = 0x02,      // 命令帧 - 发送数据后需要应答，没收到结果会报错

    ACK_NO_RESPONSE = 0x20,      // 应答帧 - 不需要应答 (00100000)
    ACK_RESPONSE_OR_NOT = 0x21,  // 应答帧 - 需要应答，没收到结果不报错 (00100001)
    ACK_WAIT_RESULT = 0x22       // 应答帧 - 需要应答，没收到结果会报错 (00100010)
} cmd_type_t;
```

因此，若要支持某个命令帧或应答帧的创建，应在 `creator` 中实现创建功能；若要支持解析，需在 `parser` 中编写解析功能。

除此之外，`send_command` 函数会根据帧类型决定是否阻塞等待数据返回，适用于发送-接收和只发送的场景。如果需要直接接收数据，则应调用 `data_wait_for_result_by_cmd` 函数。

### 修改回调函数

本程序主要在这几个地方使用了回调函数：

- **data** 数据层中的 `receive_camera_notify_handler`：在接收到 BLE 通知后调用，用于接收相机发送的数据。

- **status_logic** 中的 `update_camera_state_handler`：由 `data.c` 的 `receive_camera_notify_handler` 调用，用于更新相机的状态信息。

- **connect_logic** 中的 `receive_camera_disconnect_handler`：在 BLE 断开连接事件后调用，用于处理意外重连和主动断开连接等状态变化。

- **light_logic** 中的 `led_state_timer_callback` 和 `led_blink_timer_callback`：用于根据相应状态变化控制 RGB 灯的显示（定时器优先级默认为 1）。

### 定义按键功能

在 `key_logic` 中，为 BOOT 按键配置了长按和单击事件，并实现了相应的逻辑操作。同时，可以在此处添加更多按键和功能函数。按键扫描任务的优先级被配置为 2，需要注意的是，如果存在其他频繁执行的任务，应合理调整优先级配置，否则可能导致按键响应不灵敏或失效。

### 添加休眠功能示例

阅读完以上文档后，你可以开始尝试新增一个新功能：单击 BOOT 按键让相机休眠。

具体示例参阅文档：[添加相机休眠功能-示例文档](docs/add_camera_sleep_feature_example_CN.md)

## 其它参考文档

可以参考以下文档，对此 Demo 有更全面的了解：

* **Q&A**：[本 Demo 的常见问题与解答](docs/Q&A_CN.md)
* **ESP-IDF**：[ESP-IDF 官方 GitHub 仓库](https://github.com/espressif/esp-idf/)
* **LC76G GNSS Module**：[LC76G GNSS Module - Waveshare Wiki](https://www.waveshare.net/wiki/LC76G_GPS_Module)
* **ESP32-C6-WROOM-1**：[ESP32-C6-DevKitC-1 v1.2 - ESP32-C6 用户指南](https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html)

## 关于 PR

DJI 开发团队始终致力于提升您的开发体验，也欢迎您的贡献，但 PR 代码审查可能会有所延迟。如果您有任何疑问，欢迎通过电子邮件与我们联系。

