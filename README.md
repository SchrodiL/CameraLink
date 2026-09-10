# Osmo Action GPS Bluetooth Remote Controller (ESP32-C6-Example)

![](https://img.shields.io/badge/version-V1.0.0-red.svg) ![](https://img.shields.io/badge/platform-rtos-blue.svg) ![](https://img.shields.io/badge/license-MIT-purple.svg)

<p align="center">
  <br>English | <a href="README_CN.md">中文</a>
</p>

## Introduction

This demo provides a set of code running on the ESP32-C6 development board (based on the ESP-IDF framework), demonstrating how to parse, process, and send the DJI R SDK protocol to control the camera. The example program implements basic remote control functions, including: long-pressing the BOOT button to connect to the nearest (compatible) Osmo Action / Osmo 360 device, single-clicking to control recording, quickly switching modes, and pushing GPS data based on the LC76G GNSS module. Additionally, the program dynamically adjusts the RGB LED display based on the device's status.

Before reading this document and the code, it is recommended to first review the [Getting Started Guide](docs/getting_started_guide.md).

## Key Features

- **Protocol Parsing**: The protocol layer demonstrates how to parse the DJI R SDK protocol, which is **platform-independent and easy to port to other development platforms**.
- **GPS Data Push**: Collect GPS data at a 10Hz frequency using the LC76G GNSS module, parse it, and push it to the camera in real time.
- **Button Support**: Supports single-click (start/stop recording) and long-press (search for and connect to the nearest camera) operations. In the program, the handling of the button operations is managed by `key_logic`.
- **RGB LED Support**: Monitors the system status in real time and dynamically adjusts the RGB LED color based on status changes.
- **Other Features**: Switch the camera to a specific mode, quick switch mode (QS), subscribe to camera status, query camera version, and more.

## Development Environment

**Software**: ESP-IDF v6.0

**Hardware**:

- ESP32-C6-WROOM-1
- LC76G GNSS Module
- DJI Osmo 360 or DJI Osmo Action 6 / 5 Pro / 4

The hardware connection involves the connection between the ESP32-C6-WROOM-1 and the LC76G GNSS Module. The specific connections are as follows:

- **ESP32-C6 GPIO5** connects to **LC76G RX**
- **ESP32-C6 GPIO4** connects to **LC76G TX**
- **ESP32-C6 5V** connects to **LC76G VCC**
- **ESP32-C6 GND** connects to **LC76G GND**

Please ensure that the pins are correctly connected, especially the TX and RX pins, to ensure proper data transmission.

<img title="Hardware Wiring Diagram" src="docs/images/hardware_wiring_diagram.png" alt="Hardware Wiring Diagram" data-align="center" width="711">

## Quick Start

* Install the ESP-IDF toolchain. For installation steps, refer to the documentation below. We recommend installing the ESP-IDF extension for VSCode. You can download the plugin here: [ESP-IDF Plugin - VSCode](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)

* Next, check the `.vscode/settings.json` file in the demo to ensure that the IDF-related parameters are configured correctly.

* After setting up the environment, compile and flash the code to the development board. Use the monitor to view real-time logs. You can check the current device state by observing the RGB light status on the development board: red indicates uninitialized, yellow indicates BLE initialization complete, and the device is ready.
- When the BOOT button is long-pressed, the RGB LED flashes blue, indicating that it is searching for and connecting to the nearest Osmo Action device. A steady blue light indicates that BLE is connected, a steady green light indicates that the protocol is connected and commands can be sent and received, and a steady purple light indicates that the protocol is connected and GPS signal is available.

- When the BOOT button is clicked, the camera starts or stops recording. During long recording sessions, the RGB LED will flash.

## Demo Structure

```
├── main/             # Application assembly (app_main + CMakeLists + web frontend)
├── core/             # Protocol-agnostic controller core
│                     #   camera_state / camera_backend / controller / pairing / ble_common
├── adapters/         # Camera protocol backends (each implements camera_backend_t)
│   ├── dji/          # DJI backend (GATTC master: BLE / data / protocol / command / connect / status)
│   └── insta360/     # insta360 backend (GATTS slave)
├── services/         # Protocol-agnostic features (GPS / OSD / channel map / profile / key / light / WebUI)
├── shared/           # Shared utilities & camera-domain vocabulary (NVS helper / camera enums / CRC)
├── components/msp/   # MSP flight-controller OSD component
└── CMakeLists.txt    # Demo build file
```

- **core**: Protocol-agnostic core — unified camera state cache (`camera_state`), camera backend interface (`camera_backend`), unified control worker queue (`controller`), pairing orchestration (`pairing`), BLE stack bring-up (`ble_common`).
- **adapters**: Camera protocol backends, each implementing `camera_backend_t`; adding a new device = adding a new backend.
- **services**: Protocol-agnostic controller features (GPS parsing, MSP OSD, RC channel map, profile, key, light, WebUI).
- **shared**: Common utilities and shared camera-domain enums (mode/status/resolution/fps/eis).
- **main**: The entry point and application assembly.


## Program Startup Sequence Diagram

<img title="Program Startup Sequence Diagram" src="docs/images/sequence_diagram_of_program_startup.png" alt="Program Startup Sequence Diagram" data-align="center" width="761">

## Protocol Parsing

The following diagram illustrates the general process of frame parsing in the program:

<img title="Protocol Parsing Sequence Diagram" src="docs/images/sequence_diagram_of_protocol_parsing.png" alt="Protocol Parsing Sequence Diagram" data-align="center" width="500">

For detailed documentation, please refer to: [Protocol Parsing Documentation](docs/protocol.md)

### GPS Data Push Example

The GNRMC and GNGGA data from the LC76G GNSS module supports a maximum update frequency of 10Hz, provided that we send the corresponding command to the module:

```c
// "$PAIR050,1000*12\r\n" for 1Hz update rate
// "$PAIR050,500*26\r\n" for 5Hz update rate
// "$PAIR050,100*22\r\n" for 10Hz update rate
char* gps_command = "$PAIR050,100*22\r\n";  // (>1Hz only RMC and GGA supported)
uart_write_bytes(UART_GPS_PORT, gps_command, strlen(gps_command));
```

When parsing a large number of similar strings to extract information such as latitude, longitude, and velocity components, it is necessary to filter out invalid data. To reduce inaccuracies caused by drift, positioning errors, and other factors, it is recommended to apply filtering and other necessary processing to the GPS data before sending it. This program currently does not focus on these issues in depth, but in the future, appropriate filtering algorithms and error correction mechanisms can be introduced as needed to ensure the accuracy and reliability of the data.

Since the parsing process is frequently executed, it is important to be mindful of potential watchdog timeouts during task execution. Therefore, `vTaskDelay` has been appropriately used in the program to reset the watchdog. This program uses a simple parsing method for data pushing demonstration. Please refer to the `Parse_NMEA_Buffer` and `gps_push_data` functions in `gps_logic`.

When GPS signal is available (indicated by the solid purple RGB light), video recording will begin, and after recording ends, the corresponding data can be viewed on the DJI Mimo app dashboard.

## How to Add a Feature

**Before adding a feature, please make sure you have thoroughly read the [Protocol Parsing Documentation](docs/protocol.md) and [Data Layer Documentation](docs/data_layer.md).**

### Adding New Command Support

When sending or parsing command and response frames, you only need to follow three simple steps:

- Define the frame structure in `dji_protocol_data_structures`.

- Define the triple in `dji_protocol_data_descriptors` and provide the corresponding `creator` and `parser`. If not implemented, you can set them to `NULL`. If the parser function cannot find the corresponding `creator` or `parser`, the process of constructing or parsing the frame will stop.

- In the logic layer (`logic`), define the corresponding function, write the business logic, and call the `send_command` function in the command logic (`command_logic`).

If you add a new `.c` file in the logic layer, make sure to modify the `main/CMakeLists.txt` file.

Regarding the `send_command` function, you need to know that: in addition to passing `CmdSet`, `CmdID`, and the frame structure, you also need to pass `CmdType`, which is the frame type, defined in `enums_logic`:

```c
typedef enum {
    CMD_NO_RESPONSE = 0x00,      // Command frame - No response required after sending data
    CMD_RESPONSE_OR_NOT = 0x01,  // Command frame - Response required, no error if not received
    CMD_WAIT_RESULT = 0x02,      // Command frame - Response required, error if not received

    ACK_NO_RESPONSE = 0x20,      // Response frame - No response required (00100000)
    ACK_RESPONSE_OR_NOT = 0x21,  // Response frame - Response required, no error if not received (00100001)
    ACK_WAIT_RESULT = 0x22       // Response frame - Response required, error if not received (00100010)
} cmd_type_t;
```

Therefore, to support the creation of a command or response frame, the creation function should be implemented in the `creator`; to support parsing, the parsing function should be written in the `parser`.

Additionally, the `send_command` function will decide whether to block and wait for data return based on the frame type, which is suitable for both send-receive and send-only scenarios. If direct data reception is required, the `data_wait_for_result_by_cmd` function should be called.

### Modifying Callback Functions

This program mainly uses callback functions in the following places:

- **data** layer: `receive_camera_notify_handler`: Called after receiving a BLE notification to receive the data sent by the camera.

- In **status_logic**: `update_camera_state_handler`: Called by `data.c`'s `receive_camera_notify_handler` to update the camera's status information.

- In **connect_logic**: `receive_camera_disconnect_handler`: Called after a BLE disconnect event to handle unexpected reconnections and active disconnections, as well as state changes.

- In **light_logic**: `led_state_timer_callback` and `led_blink_timer_callback`: Used to control the RGB LED display based on corresponding state changes (the default timer priority is 1).

### Defining Button Functions

In `key_logic`, long-press and single-click events are configured for the BOOT button, with corresponding logic operations implemented. More buttons and functions can be added here. The button scanning task is configured with a priority of 2. It is important to adjust the priority appropriately if other frequently executed tasks exist, as improper priority configuration may lead to unresponsive or non-functional buttons.

### Adding Sleep Function Example

After reading the documentation above, you can try adding a new feature: putting the camera to sleep mode with a single click of the BOOT button.

For detailed implementation, please refer to: [Add Camera Sleep Feature Example Documentation](docs/add_camera_sleep_feature_example.md)

## Reference Documents

For a more comprehensive understanding of the demo, refer to the following documents:

* **Q&A**: [FAQ_for_this_demo](docs/Q&A.md)

- **ESP-IDF**: [ESP-IDF Official GitHub Repository](https://github.com/espressif/esp-idf/)

- **LC76G GNSS Module**: [LC76G GNSS Module - Waveshare Wiki](https://www.waveshare.com/wiki/LC76G_GNSS_Module)

- **ESP32-C6-WROOM-1**: [ESP32-C6-DevKitC-1 v1.2 - ESP32-C6 User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html)

## About PR

The DJI development team is dedicated to enhancing your development experience and welcomes your contributions. However, please note that PR code reviews may take some time. If you have any questions, feel free to contact us via email.

