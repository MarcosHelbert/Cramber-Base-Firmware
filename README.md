<div align="center">

<img src="docs/img/cramber_logo.svg" width="112" alt="Cramber logo">

# Cramber DD

**Open source firmware for a direct drive sim racing wheel base**

Field oriented control · USB force feedback · 1000 Hz position reporting

[![License](https://img.shields.io/badge/license-GPL--3.0-0A0A0A?style=flat-square)](LICENSE)
[![MCU](https://img.shields.io/badge/MCU-STM32F405-0A0A0A?style=flat-square)](#hardware)
[![Build](https://img.shields.io/badge/build-PlatformIO-0A0A0A?style=flat-square)](#getting-started)
[![USB](https://img.shields.io/badge/USB-HID%20PID-0A0A0A?style=flat-square)](#features)

[Getting started](#getting-started) · [Cramber Hub](https://github.com/cramber-racing/cramber-hub) · [Factory tool](https://github.com/cramber-racing/cramber-basewheel-factory)

</div>

<br>

The Cramber DD turns a brushless motor into a force feedback steering wheel. The base appears on the PC as
a standard USB force feedback wheel, so it works with any game that supports DirectInput force feedback,
with no drivers to install. On top of the game's force it adds its own effects, such as virtual end stops
and an idle centering spring, all adjustable live from [Cramber Hub](https://github.com/cramber-racing/cramber-hub).

> [!NOTE]
> This project is in active development. Firmware and apps are updated together, so keep them on matching
> versions.

## Ecosystem

| Project | Description |
| --- | --- |
| **[cramber-basewheel](https://github.com/cramber-racing/cramber-basewheel)** | Firmware that runs on the wheel base *(this repository)* |
| **[cramber-hub](https://github.com/cramber-racing/cramber-hub)** | Desktop app to adjust force feedback and manage profiles |
| **[cramber-basewheel-factory](https://github.com/cramber-racing/cramber-basewheel-factory)** | Calibration and diagnostics tool used when building a base |

## Features

<table>
<tr>
<td valign="top" width="50%">

**Force feedback**
- Standard USB force feedback wheel (HID PID)
- Wheel position reported at 1000 Hz
- Game updates applied within 1 ms
- Steering range from 90° to 2700°

**Built-in effects**
- Virtual end stops with adjustable hardness, cushion and width
- Idle spring that centers the wheel while no game is running
- Natural damping, friction and inertia
- Torque gain and FFB detail

</td>
<td valign="top" width="50%">

**Motor control**
- Field oriented control at 21 kHz
- Current loop tuned from the measured motor parameters
- Anticogging compensation for a smooth feel at low force
- Position and velocity from a Kalman filter

**Safety**
- Supply over and under voltage protection
- Motor driver fault detection and recovery
- Hardware watchdog

</td>
</tr>
</table>

## Hardware

| | |
| --- | --- |
| **Controller board** | MKS XDrive Mini by [Makerbase](https://github.com/makerbase-motor/MKS-ODrive) (ODrive based) |
| **Microcontroller** | STM32F405RG, Cortex-M4F at 168 MHz |
| **Gate driver** | DRV8301, three phase, with current sense amplifiers |
| **Encoder** | ABZ quadrature encoders only, with index (e.g. MT6835) |

Encoder resolution, motor parameters and voltage limits are factory parameters, set with the factory tool
to match your motor and power supply.

## Getting started

### 1. Install the tools

- [Visual Studio Code](https://code.visualstudio.com/) with the [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- An **ST-Link** programmer (V2 or V3)

### 2. Build

Clone the repository and open the folder in VS Code. On the first build PlatformIO downloads the compiler
and the libraries automatically.

Choose the environment in the PlatformIO toolbar and click **Build**:

| Environment | When to use it |
| --- | --- |
| `genericSTM32F405RG_release` | Normal use. Required by Cramber Hub and the factory tool |
| `genericSTM32F405RG_debug` | Development. Adds a USB serial log in place of the configuration interface |

### 3. Flash

Wire the ST-Link to the board's SWD header:

| ST-Link | Board |
| --- | --- |
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |

Power the board from its supply. The firmware has two parts: a small bootloader, flashed once, and the
application, which can later be updated from Cramber Hub without a programmer.

**First time on a board:** in the PlatformIO tasks, run **Custom → Upload full firmware**. It builds
`cramber-dd-v<version>.bin`, with the bootloader and the application, and flashes it.

**Afterwards:** click **Upload** to flash only the application, or install the firmware from Cramber Hub
(Wheel Base → System → Update firmware). Calibration and settings are kept either way.

**Custom → Build full firmware** only writes the file, to share or publish. The same
`cramber-dd-v<version>.bin` works with any ST-Link tool (flash it at `0x08000000`) and with Cramber Hub, which
takes only the application from it.

The base restarts with the new firmware and shows up on the PC as a game controller.

<details>
<summary>Using the command line instead</summary>

```sh
pio run -e genericSTM32F405RG_release                              # build
pio run -e genericSTM32F405RG_release -t upload_full_firmware      # first time on a board
pio run -e genericSTM32F405RG_release -t build_full_firmware       # write cramber-dd-v<version>.bin
pio run -e genericSTM32F405RG_release -t upload                    # application only
```

</details>

### 4. Calibrate

A freshly flashed base keeps force feedback disabled until it's calibrated. Open the
[factory tool](https://github.com/cramber-racing/cramber-basewheel-factory) and follow the **Setup** tab:
motor, find index, encoder, center, anticogging and save.

### 5. Drive

Install [Cramber Hub](https://github.com/cramber-racing/cramber-hub) to set the steering range and the force
feedback, then start a game. To check the wheel in Windows, press `Win + R` and run `joy.cpl`.

## License

Licensed under the [GPL-3.0](LICENSE). Third party libraries keep their own licenses:
[FreeRTOS](https://www.freertos.org/) (MIT), [TinyUSB](https://github.com/hathach/tinyusb) (MIT) and
[libopencm3](https://libopencm3.org/) (LGPL-3.0).
