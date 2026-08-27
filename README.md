# balance_bot

A self-balancing two-wheeled robot on a BeagleBone Blue. Flown from an FrSky SBUS
transmitter, with live telemetry, tuning and tune analysis in a browser-based
dashboard over WebSocket.

![Platform](https://img.shields.io/badge/platform-BeagleBone%20Blue-blue)
![Language](https://img.shields.io/badge/language-C-lightgrey)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Features

- **Cascade control** — pitch (angle, PID), yaw (heading, PID), position hold (encoder-based, zone-scheduled — not a PID)
- **SBUS input** — FrSky R-XSR receiver via BeagleBone UART with custom 115200 baud driver and signal inverter circuit
- **Xbox controller input** — hot-plug via `/dev/input/js0`
- **Cat following mode** — vision input from Raspberry Pi 5 running a Hailo-8L NPU, received over UART
- **Web dashboard** — telemetry, live tuning, graphs, RC channel mapping, pose/position controls and tune analysis, served from the BBB
- **Tune analysis** — every recording scored for trim, ringing, saturation and station-keeping, in the browser and from the CLI
- **Live ncurses display** — SBUS channels, PID state, encoders, IMU, motors, system status
- **IPC bridge** — Unix domain socket (`/tmp/balance_bot.sock`) to Node.js WebSocket server
- **RoboClaw motor driver** — packet serial, duty/velocity/velocity+accel modes, hardware e-stop on GPIO1_25 (resolved at runtime, not hardcoded)
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

Drive and turn channels are **configurable at runtime** from the dashboard's RC
tab and persisted to `robot.conf`; the table below is only the default.

| Channel | Switch | Function |
|---------|--------|----------|
| CH1 Ail | Right stick X | Yaw / turn (default `turn_channel`) |
| CH2 Ele | Right stick Y | Forward / back (default `drive_channel`) |
| CH5 SA | 3-pos | Arm / Disarm |
| CH6 SB | 3-pos | Kill switch |
| CH10 SD | 3-pos | Speed mode (slow/normal/sport) |

Drive is held at zero until the drive channel has been seen near centre once, so
a transmitter left out of trim cannot command movement the instant it links.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    balance_bot (C)                       │
│                                                         │
│  IMU interrupt @ 100Hz                                  │
│  └── imu_apply_transform()                              │
│  └── pitch PID  → motor mixing → RoboClaw               │
│  └── yaw PID    ┘                                       │
│                                                         │
│  Main loop @ 100Hz                                      │
│  └── sbus_update() / xbox_update()                      │
│  └── position hold (encoder-based)                      │
│  └── telemetry_update()                                 │
│  └── ipc_broadcast_telemetry()  ──→ Unix socket         │
└────────────────────┬────────────────────────────────────┘
                     │ /tmp/balance_bot.sock
┌────────────────────▼────────────────────────────────────┐
│               server/server.js (Node.js)                 │
│   Unix socket client  ←→  WebSocket server :8675        │
└────────────────────┬────────────────────────────────────┘
                     │ WebSocket ws://boneblue-0:8675
                     ▼
              bbot_dashboard_v6.html
              http://boneblue-0:8888
```

### Source Files

| File | Description |
|------|-------------|
| `src/main.c` | Entry point, argument parsing, subsystem init |
| `src/robot.c` | IMU interrupt, PID loop, motor output, main run loop |
| `src/pid.c` | Generic PID controller with anti-windup |
| `src/robot_config.c` | Single sectioned `robot.conf`: gains, position, IMU, SBUS, motor. Atomic writer, migrates from the old files once |
| `src/imu_config.c` | IMU axis remapping, calibration offsets, `imu_offsets_calibrate()` |
| `src/ipc_server.c` | Unix socket server, JSON telemetry, command parsing |
| `src/telemetry.c` | Telemetry data collection from all subsystems |
| `src/mpu_dmp.c` | Custom MPU-9250 DMP driver (no librobotcontrol dependency) |
| `src/roboclaw.c` | RoboClaw packet-serial driver |
| `src/motor_hal_roboclaw.c` | Motor HAL backend: RoboClaw duty/velocity/velocity+accel |
| `src/roboclaw_estop.c` | Hardware e-stop on GPIO1_25, resolved from the GPIO controller at runtime (sysfs numbering shifts between kernels) |
| `src/input_sbus.c` | SBUS frame parser, custom baud rate, channel decode |
| `src/input_xbox.c` | Xbox controller via Linux joystick API |
| `src/display.c` | ncurses live display, threaded redraw |

| Tool | Description |
|------|-------------|
| `tools/analyze_tune.py` | Tune-quality report from a telemetry CSV; compares runs |
| `tools/trim_from_log.py` | Balance trim from measured drift, not by eye |
| `tools/check_link.sh` | Finds undefined symbols without linking |
| `tools/bbot_watch.py` | 1 Hz system recorder for diagnosing lockups |
| `config/bashrc` | The shell environment: aliases, service control, `bhelp` / `ahelp`. Install with `./config/install_bashrc.sh --apply` |

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
2. Clears the RoboClaw e-stop
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
  -p <file>     Config file                            (default: robot.conf)
  -u <device>   UART device for SBUS or EXT input
  -m <device>   RoboClaw UART device                  (default: /dev/ttyO1)
  -B <baud>     RoboClaw baud rate                    (default: 460800)
  -d <block>    Enable display block (repeatable):
                  sbus | pid | enc | imu | mot | sys | all | none
  -h            Help
```

---

## Web Dashboard

A single-file HTML dashboard (`web/bbot_dashboard_v6.html`) provides full control and telemetry from any browser on the network.

```
web/
  bbot_dashboard_v6.html   # dashboard UI (current)
  serve_web.py             # stdlib HTTP server, no dependencies
```

`start.sh` launches `serve_web.py` automatically. Navigate to:

```
http://boneblue-0:8888/bbot_dashboard_v6.html
```

To run standalone without `start.sh`:

```bash
ssh debian@boneblue-0 "cd ~/balance_bot && python3 web/serve_web.py &"
```

### Dashboard tabs

| Tab | Contents |
|-----|----------|
| Control | ARM/DISARM, E-STOP, CLR ESTOP, Zero IMU, Zero Encoders, mode/motor mode pickers, encoder readout, MJPEG video with cat overlay |
| PID | Pitch / Position / Yaw picker. Kp/Ki/Kd for pitch and yaw; position shows its zone config instead, since it has no gains. Balance trim lives on the pitch card |
| Graph | Live scrolling plots, dual-axis combined view, series toggle, CSV record/download |
| Claw | RoboClaw drive mode, QPPS, polarity, velocity PID |
| RC | Live SBUS channels and switches, channel mapping / scale / invert / deadband, pose (pitch, yaw) and position controls |
| Tune | Tune-quality report for the last recording, saved run history, and A/B comparison including a config diff |
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

## Configuration

Everything tunable lives in one sectioned file, `robot.conf`, written atomically
and loaded at startup. It replaces `pidconfig.txt` and
`/etc/balance_bot_imu.conf`; if those are present and `robot.conf` is not, they
are migrated once and then unused.

**Only pitch and yaw have gains.** Position hold is not a PID — it is a
zone-scheduled controller configured entirely by its zones and scales.

```ini
[pitch]
kp = 0.070
ki = 0.020
kd = 0.005

[yaw]
kp = 0.010
ki = 0.005
kd = 0.000

[position]
zone_a = 8000          # outer threshold, ticks. Contract is A > B > C
zone_b = 4000
zone_c = 500
scale_a = 60.000       # tick error / scale = lean bias, deg. Larger = weaker
scale_b = 80.000
scale_c = 200.000
scale_d = 50.000       # inside zone_c, the tightest hold
vel_scale_stop = 5.000
max_correction = 5.000
max_angle_rate = 0.100 # deg per tick, rate-limits the correction
back_to_spot = 0       # 1 = chase the target, 0 = loose hold
drive_mode = 0         # 0 = stick commands lean, 1 = stick moves the target
drive_rate = 300.0     # ticks/s at full stick in target mode
runaway_limit = 300    # max |target - pos|, anti-windup

[imu]
pitch_offset = 98.5500 # where upright is, from calibration
pitch_axis = 1

[sbus]
drive_channel = 3
turn_channel = 1
drive_scale = 0.250
turn_rate = 60.0       # deg/s of heading at full stick
require_center = 1

[motor]
mode = 0               # 0 = duty, 1 = velocity, 2 = velocity + accel
pol_l = -1.0
enc_pol_l = -1.0
```

Section names changed with the controller rename; `[balance]` and `[steering]`
are still accepted so an older file keeps loading.

### Backing it up

`robot.conf` exists only on the SD card and is excluded from rsync, so a sync
cannot overwrite a live calibration. That also means nothing backs it up:

```bash
make save-config     # on the bot: copies robot.conf into config/machine/
                     # then commit config/machine/
make install-config  # on a fresh board: restores it, never overwrites
```

---

## Balance Trim

The single most misleading fault on this machine. The balance point is where the
centre of mass sits over the tyre contact patch — not something you can see, and
with the battery and controller distributed unevenly it is not the geometric
centreline. Worse, the pitch integrator winds up to compensate, so the robot
balances happily at the wrong angle and never falls. The error shows up only as
a slow creep, which reads as a drive fault.

So do not set it by eye. Measure the drift:

```bash
# record ~30s of balancing from the dashboard, then
python3 tools/trim_from_log.py <log.csv>
```

```
net travel -11 ticks = -0.04 m over 80.9s
TRIMMED. Mean drift +0.03 is inside +/-0.3
```

If it reports creeping, nudge the trim by the suggested amount and record again.
Two or three rounds is normal. The dashboard's Tune tab reports the same thing
without the export.

---

## Tune Analysis

Every recording is scored, in the browser on the Tune tab and from the CLI:

```bash
python3 tools/analyze_tune.py <log.csv>              # full report
python3 tools/analyze_tune.py before.csv after.csv   # what changed
python3 tools/analyze_tune.py *.csv --brief          # one line each
```

| Section | Question it answers |
|---------|--------------------|
| Trim | Does it hold station, or creep? |
| Balance | How tightly does it track upright, and is it running out of authority? |
| Ringing | Is there one dominant oscillation, and how big? |
| Position | How well does station-keeping hold? |
| Effort | How hard is it working, and is one side working harder? |
| Health | Did the control loop keep up — is this run even trustworthy? |

Each metric prints against a stated threshold, followed by a prioritised list of
what to try next. Comparison mode also diffs the config the two runs were
recorded under, so "did that help?" and "what did I actually change?" are
answered together.

---

## IPC Command Reference

Commands are JSON sent over WebSocket to `server.js`, which forwards them to the Unix socket:

| Command | JSON |
|---------|------|
| Arm / disarm | `{"type":"arm","value":true}` |
| E-stop (assert) | `{"type":"e_stop"}` |
| Clear e-stop (WriteNVM reset) | `{"type":"reset_estop"}` |
| Set mode | `{"type":"set_mode","value":1}` (0=idle 1=balance 2=ext 3=manual) |
| Set gains | `{"type":"set_pid","controller":"pitch","kp":40,"ki":0,"kd":5}` |
| Enable controller | `{"type":"set_controller","controller":"yaw","enabled":true}` |
| Save config | `{"type":"save_pid"}` |
| Zero IMU + encoders | `{"type":"zero_imu"}` |
| Zero encoders only | `{"type":"zero_encoders"}` |
| Position hold config | `{"type":"set_pos_config","zone_a":8000,"scale_d":80,...}` |
| Motor config | `{"type":"set_motor_config","mode":0}` |
| RC mapping | `{"type":"set_sbus_config","drive_channel":3,"drive_scale":0.25}` |
| Nudge pose / position | `{"type":"nudge","axis":"pose","delta":0.1}` — axes: `pitch` (trim), `pose` (lean), `yaw`, `fwd` |
| Telemetry options | `{"type":"set_telemetry","encoders":true,"pid_states":true}` |

Controller names are `pitch`, `position` and `yaw`. The older `balance` and
`steering` are still accepted, so an out-of-date browser tab keeps working.

Send one by hand with:

```bash
echo '{"type":"set_pos_config","drive_mode":1}' | socat - UNIX-CONNECT:/tmp/balance_bot.sock
```

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