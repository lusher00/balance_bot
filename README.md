# balance_bot

A self-balancing two-wheeled robot running on a BeagleBone Blue. Controlled via FrSky SBUS RC receiver or Xbox controller, with real-time telemetry and PID tuning streamed to an iPhone companion app or web dashboard over WebSocket.

![Platform](https://img.shields.io/badge/platform-BeagleBone%20Blue-blue)
![Language](https://img.shields.io/badge/language-C-lightgrey)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Features

- **Cascade PID control** — D1 balance (angle), D3 steering (yaw), D2 position hold (encoder-based)
- **SBUS input** — FrSky R-XSR receiver via BeagleBone UART with custom 115200 baud driver and signal inverter circuit
- **Xbox controller input** — hot-plug via `/dev/input/js0`
- **Cat following mode** — vision input from Raspberry Pi 5 running a Hailo-8L NPU, received over UART
- **iPhone companion app** — real-time telemetry, live PID tuning, dual-axis graphs, arm/disarm, D2 pos config
- **Web dashboard** — same feature set as the iPhone app, served from the BBB, accessible from any browser on the network
- **Live ncurses display** — SBUS channels, PID state, encoders, IMU, motors, system status
- **IPC bridge** — Unix domain socket (`/tmp/balance_bot.sock`) to Node.js WebSocket server
- **RoboClaw motor driver** — packet serial, duty/velocity/velocity+accel modes, hardware e-stop via GPIO57
- **Systemd integration** — `make install` deploys and manages both services

---

## Hardware

| Component | Part |
|-----------|------|
| Controller | BeagleBone Blue |
| IMU | Onboard MPU-9250 via custom DMP driver |
| Motor driver | RoboClaw (packet serial, `/dev/ttyO1` at 460800 baud) |
| Motors | RS-555 with 5.2:1 planetary gearbox |
| Encoders | Quadrature, 145.1 PPR, ~3.36mm/tick |
| Wheels | 155mm diameter |
| RC Receiver | FrSky R-XSR (SBUS) |
| Transmitter | Jumper T16 (OpenTX) |
| Vision coprocessor | Raspberry Pi 5 + Hailo-8L NPU |

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
│  └── imu_apply_transform()                              │
│  └── D1: balance PID  → motor mixing → RoboClaw        │
│  └── D3: steering PID ┘                                 │
│                                                         │
│  Main loop @ 100Hz                                      │
│  └── sbus_update() / xbox_update()                      │
│  └── D2: position hold (encoder-based)                  │
│  └── telemetry_update()                                 │
│  └── ipc_broadcast_telemetry()  ──→ Unix socket         │
└────────────────────┬────────────────────────────────────┘
                     │ /tmp/balance_bot.sock
┌────────────────────▼────────────────────────────────────┐
│               server/server.js (Node.js)                 │
│   Unix socket client  ←→  WebSocket server :8675        │
└────────────────────┬────────────────────────────────────┘
                     │ WebSocket ws://boneblue-0:8675
              ┌──────┴──────┐
              ▼             ▼
  BBotTuneHUD (iOS)    BBotHUD (web)
  SwiftUI app          http://boneblue-0:8888
```

### Source Files

| File | Description |
|------|-------------|
| `src/main.c` | Entry point, argument parsing, subsystem init |
| `src/robot.c` | IMU interrupt, PID loop, motor output, main run loop |
| `src/pid.c` | Generic PID controller with anti-windup |
| `src/pid_config.c` | Load/save/apply PID gains and pos_config from `pidconfig.txt` |
| `src/imu_config.c` | IMU axis remapping, calibration offsets, `imu_offsets_calibrate()` |
| `src/ipc_server.c` | Unix socket server, JSON telemetry, command parsing |
| `src/telemetry.c` | Telemetry data collection from all subsystems |
| `src/mpu_dmp.c` | Custom MPU-9250 DMP driver (no librobotcontrol dependency) |
| `src/roboclaw.c` | RoboClaw packet-serial driver |
| `src/motor_hal_roboclaw.c` | Motor HAL backend: RoboClaw duty/velocity/velocity+accel |
| `src/roboclaw_estop.c` | Hardware e-stop via GPIO57 (active-low, RoboClaw latches) |
| `src/input_sbus.c` | SBUS frame parser, custom baud rate, channel decode |
| `src/input_xbox.c` | Xbox controller via Linux joystick API |
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

### Build and install

```bash
git clone https://github.com/lusher00/balance_bot
cd balance_bot
make
sudo make install
```

`make install` stops the running service, copies the binary to `/usr/local/bin/`, and restarts.

---

## Running

### Quick start (bench session)

```bash
./start.sh                    # balance only, all display blocks
./start.sh -i sbus            # SBUS input via R-XSR on /dev/ttyO5
./start.sh -i sbus -d pid     # SBUS + PID display panel
./start.sh -i xbox            # Xbox controller
```

`start.sh` handles the full startup sequence:
1. Kills any stale processes
2. Clears the RoboClaw e-stop (`estop_clear.sh`)
3. Resets the RoboClaw via WriteNVM (`roboclaw_reset.py`) — required after any e-stop latch
4. Starts `server.js` (WebSocket bridge) in background
5. Starts `serve_web.py` (web dashboard) in background on port 8888
6. Runs `balance_bot` in the foreground (ncurses takes over the terminal)

### Systemd service (auto-start on boot)

```bash
sudo cp systemd/balance_bot.service /etc/systemd/system/
sudo cp systemd/balance_bot_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable balance_bot balance_bot_server
```

### Manual

```bash
sudo balance_bot [options]

Options:
  -i <mode>     Input mode: sbus | xbox | ext | none  (default: none)
  -p <file>     PID config file                       (default: pidconfig.txt)
  -u <device>   UART device for SBUS or EXT input
  -m <device>   RoboClaw UART device                  (default: /dev/ttyO1)
  -B <baud>     RoboClaw baud rate                    (default: 460800)
  -d <block>    Enable display block (repeatable):
                  sbus | pid | enc | imu | mot | sys | all | none
  -h            Help
```

---

## Web Dashboard

A single-file HTML dashboard (`web/bbot_dashboard.html`) provides the same controls as the iPhone app from any browser on the network.

```
web/
  bbot_dashboard.html   # dashboard UI
  serve_web.py          # stdlib HTTP server, no dependencies
```

`start.sh` launches `serve_web.py` automatically. Navigate to:

```
http://boneblue-0:8888/bbot_dashboard.html
```

To run standalone without `start.sh`:

```bash
ssh debian@boneblue-0 "cd ~/balance_bot && python3 web/serve_web.py &"
```

### Dashboard tabs

| Tab | Contents |
|-----|----------|
| Control | ARM/DISARM, E-STOP, CLR ESTOP, Zero IMU, Zero Encoders, mode/motor mode pickers, encoder readout, MJPEG video with cat overlay |
| PID | D1/D2/D3 picker, Kp/Ki/Kd sliders with ×1/×10/×100 step, full D2 pos_config (all 13 fields) |
| Graph | Live scrolling plots, D1+D2 dual-axis combined view, series toggle, CSV record/download |
| Debug | Syntax-highlighted raw telemetry JSON |
| Settings | BBB IP/port, Pi 5 IP, video URL, telemetry option toggles |

### ARM button states

| Color | Meaning |
|-------|---------|
| 🟢 Green | Disarmed — ready to arm |
| 🟡 Yellow | Armed but out of bounds (>14°) — no motor output |
| 🔴 Red | Armed and balancing |

### E-stop recovery

After a fall the RoboClaw latches its e-stop internally. To recover without restarting:

1. Press **CLR ESTOP** in the dashboard (sends `reset_estop` command)
2. Wait ~2 seconds while the BBB runs the WriteNVM reset sequence
3. Press **ARM** once the button goes green

---

## PID Configuration

Gains are loaded from `pidconfig.txt` at startup and can be updated live from either client without restarting.

```
0                         # legacy holdPosition flag (unused)
0.000                     # balance_angle / theta_offset trim (deg)
0.050 0.010 0.005         # D1 balance:  Kp Ki Kd
0.000 0.000 0.000         # D2 drive:    Kp Ki Kd (unused — zone-based controller)
0.010 0.010 0.000         # D3 steering: Kp Ki Kd

# pos_config
zone_a=8000.0
zone_b=4000.0
zone_c=500.0
scale_a=60.0
scale_b=80.0
scale_c=200.0
scale_d=300.0
...
```

---

## IMU Calibration

The balance angle offset is stored in `/etc/balance_bot_imu.conf` (fallback: `~/balance_bot_imu.conf`).

```bash
# Zero with robot balanced upright — use Zero IMU in either client
# Or directly:
echo "pitch_offset 0.0" | sudo tee /etc/balance_bot_imu.conf
```

The **Zero IMU** command sets `pitch_offset = current_pitch_deg` so `theta = 0` at the robot's current physical position. It also resets encoders and saves all config.

---

## IPC Command Reference

Commands are JSON sent over WebSocket to `server.js`, which forwards them to the Unix socket:

| Command | JSON |
|---------|------|
| Arm / disarm | `{"type":"arm","value":true}` |
| E-stop (assert) | `{"type":"e_stop"}` |
| Clear e-stop (WriteNVM reset) | `{"type":"reset_estop"}` |
| Set mode | `{"type":"set_mode","value":1}` (0=idle 1=balance 2=ext 3=manual) |
| Set PID gains | `{"type":"set_pid","controller":"D1_balance","kp":40,"ki":0,"kd":5}` |
| Enable controller | `{"type":"set_controller","controller":"D1_balance","enabled":true}` |
| Save config | `{"type":"save_pid"}` |
| Zero IMU + encoders | `{"type":"zero_imu"}` |
| Zero encoders only | `{"type":"zero_encoders"}` |
| Set D2 pos config | `{"type":"set_pos_config","zone_a":8000,"scale_d":80,...}` |
| Set motor config | `{"type":"set_motor_config","mode":0}` |
| Telemetry options | `{"type":"set_telemetry","encoders":true,"pid_states":true}` |
| Request config snapshot | `{"type":"get_config"}` |

---

## Node.js Bridge (`server/`)

Bridges WebSocket clients to the C process via Unix domain socket.

```bash
cd server && npm install && node server.js
```

Managed by `balance_bot_server.service`.

---

## License

MIT