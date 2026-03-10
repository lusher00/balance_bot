# balance_bot

A self-balancing two-wheeled robot running on a BeagleBone Blue. Controlled via FrSky SBUS RC receiver or Xbox controller, with real-time telemetry and PID tuning streamed to an iPhone companion app over WebSocket.

![Platform](https://img.shields.io/badge/platform-BeagleBone%20Blue-blue)
![Language](https://img.shields.io/badge/language-C-lightgrey)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Features

- **Cascade PID control** — D1 balance (angle), D3 steering (yaw), D2 drive (position, optional)
- **SBUS input** — FrSky R-XSR receiver via BeagleBone UART with custom 115200 baud driver and signal inverter circuit
- **Xbox controller input** — hot-plug via `/dev/input/js0`
- **Cat following mode** — vision input from Raspberry Pi 5 running a Hailo-8L NPU, received over UART
- **iPhone companion app** — real-time telemetry, live PID tuning, 3D IMU visualization, arm/disarm control
- **Live ncurses display** — SBUS channels, PID state, encoders, IMU, motors, system status
- **IPC bridge** — Unix domain socket (`/tmp/balance_bot.sock`) to Node.js WebSocket server
- **Systemd integration** — `make install` deploys and manages both services

---

## Hardware

| Component | Part |
|-----------|------|
| Controller | BeagleBone Blue |
| IMU | Onboard MPU-9250 via DMP |
| Motors | DC gearmotors via onboard H-bridge |
| Encoders | Quadrature, 2400 ticks/rev |
| RC Receiver | FrSky R-XSR (SBUS) |
| Transmitter | Jumper T16 (OpenTX) |
| Vision | Raspberry Pi 5 + Hailo-8L NPU |

### SBUS Wiring

The R-XSR outputs inverted SBUS (115200 baud, 8E2, active-low). An NPN transistor (2N3904) with 10kΩ pull-up re-inverts the signal before the BeagleBone UART RX pin.

```
R-XSR SBUS → 2N3904 base (10kΩ) → BeagleBone P9.26 (UART1 RX)
R-XSR 5V/GND → BeagleBone 5V / GND
```

### RC Channel Mapping (AETR / OpenTX)

| Channel | Switch | Function |
|---------|--------|----------|
| CH1 Ail | Right stick X | Yaw / turn |
| CH2 Ele | Right stick Y | Forward / back |
| CH5 SA | 3-pos | Arm / Disarm |
| CH6 SB | 3-pos | Kill switch |
| CH10 SD | 3-pos | Speed mode (slow/normal/sport) |

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    balance_bot (C)                       │
│                                                         │
│  IMU interrupt @ 100Hz                                  │
│  └── imu_config_apply_transform()                       │
│  └── D1: balance PID  → motor mixing → rc_motor_set()  │
│  └── D3: steering PID ┘                                 │
│                                                         │
│  Main loop @ 100Hz                                      │
│  └── sbus_update() / xbox_update()                      │
│  └── telemetry_update()                                 │
│  └── ipc_broadcast_telemetry()  ──→ Unix socket         │
│  └── display_update()                                   │
└────────────────────┬────────────────────────────────────┘
                     │ /tmp/balance_bot.sock
┌────────────────────▼────────────────────────────────────┐
│               server/server.js (Node.js)                 │
│   Unix socket client  ←→  WebSocket server :8675        │
└────────────────────┬────────────────────────────────────┘
                     │ WebSocket ws://beaglebone:8675
┌────────────────────▼────────────────────────────────────┐
│              Upright iOS App (Swift/SwiftUI)             │
│   Live telemetry · PID tuning · 3D IMU · Arm/Disarm     │
└─────────────────────────────────────────────────────────┘
```

### Source Files

| File | Description |
|------|-------------|
| `src/main.c` | Entry point, argument parsing, subsystem init |
| `src/robot.c` | IMU interrupt, PID loop, motor output, main run loop |
| `src/pid.c` | Generic PID controller with anti-windup |
| `src/pid_config.c` | Load/save/apply PID gains from `pidconfig.txt` |
| `src/imu_config.c` | IMU axis remapping and calibration offsets |
| `src/ipc_server.c` | Unix socket server, JSON telemetry, command parsing |
| `src/telemetry.c` | Telemetry data collection from all subsystems |
| `src/input_sbus.c` | SBUS frame parser, custom baud rate, channel decode |
| `src/input_xbox.c` | Xbox controller via Linux joystick API |
| `src/cat_tracker.c` | Cat position input from RPi5 over UART |
| `src/display.c` | ncurses live display, threaded redraw |

---

## Building

### Dependencies

```bash
# librobotcontrol (BeagleBone Blue)
sudo apt install librobotcontrol librobotcontrol-dev

# ncurses
sudo apt install libncurses5-dev
```

### Build

```bash
git clone https://github.com/yourusername/balance_bot
cd balance_bot
make
```

### Install as systemd service

```bash
make install
```

This stops the running service, copies the binary to `/usr/local/bin/`, and restarts. Both `balance_bot.service` and `balance_bot_server.service` unit files are in `systemd/`.

```bash
# Install service units
sudo cp systemd/balance_bot.service /etc/systemd/system/
sudo cp systemd/balance_bot_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable balance_bot balance_bot_server
```

---

## Usage

```bash
sudo balance_bot [options]

Options:
  -i <mode>     Input mode: cat | xbox | sbus | none  (default: cat)
  -p <file>     PID config file                       (default: pidconfig.txt)
  -u <device>   UART device for SBUS                  (default: /dev/ttyO4)
  -d <block>    Enable display block (repeatable):
                  sbus | pid | enc | imu | mot | sys | all | none
  -h            Help

Examples:
  sudo balance_bot -i sbus -d all       # SBUS input, all display blocks
  sudo balance_bot -i none              # Balance only, no RC input
  sudo balance_bot -i xbox /dev/input/js0
```

### Without an RC Controller

When no controller is connected, arm and control via the iPhone app:

1. Place robot upright
2. Open Upright app → tap **ARM** (rejected if lean angle > ~14°)
3. Adjust PID gains live from the PID Tuning tab
4. Tap **DISARM** before it falls

---

## PID Configuration

Gains are loaded from `pidconfig.txt` at startup and can be updated live from the iPhone app without restarting.

```
# pidconfig.txt
0         # version
0.01      # dt
40.0 0.0 5.0    # D1 balance:  Kp Ki Kd
20.0 0.5 2.0    # D2 drive:    Kp Ki Kd
15.0 0.0 1.5    # D3 steering: Kp Ki Kd
```

---

## IMU Calibration

```bash
# Run once with robot on flat surface
rc_calibrate_accel

# Zero angle offsets with robot balanced upright
# → Use the "Zero display" button in the Upright iOS app
# → Or call imu_config_calibrate() which writes /etc/balance_bot_imu.conf
```

---

## IPC Command Reference

Commands are JSON sent from the Node.js bridge to the Unix socket:

| Command | Example |
|---------|---------|
| Arm / disarm | `{"type":"arm","value":true}` |
| Set mode | `{"type":"set_mode","value":1}` (0=idle, 1=balance, 2=follow, 3=manual) |
| Set PID gains | `{"type":"set_pid","controller":"D1_balance","kp":40,"ki":0,"kd":5}` |
| Enable controller | `{"type":"set_controller","controller":"D1_balance","enabled":true}` |
| Set theta reference | `{"type":"set_theta_ref","value":0.05}` |
| Telemetry options | `{"type":"set_telemetry","encoders":true,"pid_states":true}` |

---

## Node.js Bridge (`server/`)

A lightweight WebSocket server that bridges the iPhone app to the C process.

```bash
cd server
npm install
node server.js
```

Managed by `balance_bot_server.service`. Starts automatically after `balance_bot.service`.

---

## License

MIT

