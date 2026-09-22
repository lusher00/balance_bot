# balance_bot

A self-balancing two-wheeled robot on a BeagleBone Blue. Flown from an FrSky SBUS
transmitter, with live telemetry, tuning and tune analysis in a browser-based
dashboard over WebSocket.

![Platform](https://img.shields.io/badge/platform-BeagleBone%20Blue-blue)
![Language](https://img.shields.io/badge/language-C-lightgrey)
![License](https://img.shields.io/badge/license-PolyForm%20Noncommercial%201.0.0-blue)

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
- **Motor current sensing and limiting** — per-motor current read back at 20 Hz with a running peak, and a hardware current limit enforced by the RoboClaw itself (see [Current limiting](#current-limiting))
- **Motion gate** — one choke point in `motor_hal_set_both()`: no drive command reaches the RoboClaw unless armed, in bounds and the IMU is live; otherwise it coasts (see [Motion gate](#motion-gate))
- **Always-on 100 Hz history** — every control tick kept in a RAM ring on the bot, downloadable after the fact (see [100 Hz history ring](#100-hz-history-ring))
- **Systemd integration** — `make install` deploys and manages the services; unit files are only reinstalled when they actually change

---

## Hardware

| Component | Part |
|-----------|------|
| Controller | BeagleBone Blue |
| IMU | Onboard MPU-9250 via custom DMP driver |
| Motor driver | RoboClaw (packet serial, `/dev/ttyO1` at 460800 baud) |
| Motors | RS-555 with 5.2:1 planetary gearbox |
| Encoders | Quadrature, 145.1 PPR, ~2.54 mm per wheel tick (enc_pos is L+R summed: 1.27 mm per unit) |
| Wheels | BaneBots 4-5/8" (117.475 mm) |
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
| `tools/wifi_prefer.sh` | Picks the best available network once, 45 s after boot. `iwd` has no priority field and does not roam between SSIDs, so the ordering has to live outside it |
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

**`start.sh` is a bench tool that replaces the systemd stack, not an addition
to it.** It stops the services, stands up its own copy of everything, and runs
the binary in the foreground so you can watch it and Ctrl-C it. Sequence:

1. Stamps `bbot_run START` into the kernel ring buffer (survives a filesystem death — read it with `dmesg`)
2. Stops `balance_bot_server` and `balance_bot`, and stops `balance_bot_web` if it was running
3. Resets the RoboClaw via WriteNVM (`roboclaw_reset.py`)
4. **Then** clears the latched e-stop (`estop_clear.sh`) — this order matters; clearing before the reset does not take
5. Starts its own `server.js` (WebSocket bridge) in background
6. Starts its own `serve_web.py` (web dashboard) in background on port 8888
7. Runs `balance_bot` in the foreground (ncurses takes over the terminal)

On exit it stamps `bbot_run END`, kills its own children, and restarts
`balance_bot_web` if it stopped it. It deliberately does **not** restart
`balance_bot` or `balance_bot_server` — Ctrl-C is how you make the bot stop,
and bringing it back under systemd would defeat that. Use `rbots` to restore
everything afterwards.

### Systemd service (auto-start on boot)

```bash
make install-units          # copies only the units that changed, reloads if any did
sudo systemctl enable balance_bot balance_bot_server
```

Five services make up a running bot. `botss` shows all of them at once; `rbots`
restarts all of them. Note that `botss` is *status*, not restart.

| Unit | What it is |
|------|------------|
| `balance_bot` | The control binary. Args come from `/etc/default/balance_bot`, not from the unit |
| `balance_bot_server` | Node WebSocket bridge. `Requires=balance_bot` |
| `balance_bot_web` | `serve_web.py` on port 8888 — the dashboard |
| `batt_monitor` | Battery supervision |
| `bbb_oled` | OLED status display |

Only `balance_bot.service` and `balance_bot_server.service` are versioned in
`systemd/` today. The other three live on the board only.

`systemd/wifi-prefer.{service,timer}` is separate and optional — a one-shot
network chooser that runs 45 s after boot.

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

### 100 Hz history ring

The control loop appends every tick to `/dev/shm/bbot_ring_0.csv` /
`bbot_ring_1.csv` — RAM, never the SD card or eMMC. Each segment is 5 MB and
they alternate, so the last ~4–7 minutes at 100 Hz are always on the bot. `t`
is seconds since boot (same clock as telemetry). Each segment starts with the
full config; any config change mid-run writes `# CHANGE t=...` and the new
config block in place.

- Dashboard: **⤓ RING** with a seconds box (0 = everything)
- Direct: `http://boneblue-0:8888/bbot_ring.csv?sec=60`

Nothing to start or stop — when something happens, download it. The 20 Hz
REC CSV likewise marks mid-recording changes with `# CHANGE` lines. Position
fields on the dashboard always show the bot's values (red only while a typed
value has not reached the bot).

### Dashboard tabs

| Tab | Contents |
|-----|----------|
| Control | ARM/DISARM, E-STOP, CLR ESTOP, Zero IMU, Zero Encoders, mode/motor mode pickers, encoder readout, MJPEG video with cat overlay |
| PID | Pitch / Position / Yaw picker. Kp/Ki/Kd for pitch and yaw; position shows its zone config instead, since it has no gains. Balance trim lives on the pitch card |
| Graph | Live scrolling plots, dual-axis combined view, series toggle, CSV record/download |
| Claw | RoboClaw drive mode, QPPS, acceleration, **current limit**, polarity, velocity PID |
| RC | Live SBUS channels and switches, channel mapping / scale / invert / deadband, pose (pitch, yaw) and position controls |
| Tune | Tune-quality report for the last recording, saved run history, and A/B comparison including a config diff |
| Debug | Syntax-highlighted raw telemetry JSON |
| Settings | BBB IP/port, Pi 5 IP, video URL, telemetry option toggles |

### Status ribbon

The header ribbon is visible on every tab and carries the things you should
never have to change tabs to see:

```
Batt 12.3V   Claw V 12.1V   Amps 0.31/0.28  pk 1.85   Angle 0.4°   Loop 100
```

`Amps` is live M1/M2 current with a running peak beside it. It is coloured
against the configured current limit, not an absolute number — amber past 75%
of the limit, red at it, where the controller is clamping and the bot is no
longer getting the torque the controller asked for. Click the segment to clear
the peaks.

Below it, the action bar (ARM, ZERO IMU, ENC 0, E-STOP, CLR ESTOP, and the two
recorders) is also always visible.

### ARM button states

| Color | Meaning |
|-------|---------|
| 🟢 Green | Disarmed — ready to arm |
| 🟡 Yellow | Armed but out of bounds (>15°) — motors cut, still armed |
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
gyro_scale = 0.000     # 0 = D from the encoder staircase; else gyro Z, scaled
                       # into phi_diff deg/s, sign included. See Yaw Derivative
                       # Source — it is measured from a log, not guessed.

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
lead_max = 0           # return-home carrot: max lead over the wheels, ticks. 0 = off
return_rate = 100.0    # ticks/s the carrot walks home (1 tick = 1.27 mm)

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
accel_qpps = 5000      # MODE 2 ONLY — ignored in modes 0 and 1
max_amps = 2.5         # per-motor hardware current limit, 0 = leave controller alone
pol_l = -1.0
enc_pol_l = -1.0

[system]
arm_at_boot = 0         # 1 = arm by itself when stood up (see Arm at boot)
oob_angle_deg = 15.0    # |theta - trim| beyond this = out of bounds, motors coast
kick_assist = 1         # 1 = allow the stand-up kick (see Stand-Up Kick)
kick_max_deg = 25.0     # the kick refuses above this |theta - trim|
kick_duty = 0.35        # duty it drives both wheels at
kick_timeout_ms = 1200  # longest single kick before it demands a re-centre
```

Section names changed with the controller rename; `[balance]` and `[steering]`
are still accepted so an older file keeps loading.

### Current limiting

`max_amps` is pushed to the RoboClaw at init, before anything can command
motion, and again whenever it changes from the dashboard. The controller
enforces it in hardware.

**This is the real overcurrent protection.** A fuse opens on I²t over seconds;
a capacitor covers microseconds; the failure mode in between is a stalled motor
holding V/R_winding for as long as the fault lasts. With no back-EMF opposing
it, a small brushed motor at 12 V and ~1 Ω pulls well into double digits, and a
balancing bot reverses constantly — on a reversal the back-EMF *adds* to the
supply across the winding. A bot driving into the floor holds that continuously.

The default of 2.5 A per motor is 5 A total, which sits under a 6 A fuse with
margin. Raise it only after watching real numbers on the ribbon.

Two things worth knowing:

- **Older RoboClaw firmware does not implement the current-limit commands
  (133/134).** `motor_hal_read_current_limit()` exists so you can confirm the
  limit actually took. If it does not read back what you set, the limit is a
  no-op.
- **`accel_qpps` only applies in mode 2.** In mode 1 every setpoint change is a
  step and the controller's velocity PID does whatever duty it takes to hit it
  immediately, so di/dt is unbounded. If current peaks look violent, mode 2
  with a sane ramp is the first lever.

A bench supply is a poor instrument here: its meter averages over hundreds of
milliseconds while the events are milliseconds, and a current-limited supply
clips the transient and then shows you the average of what it allowed through.
If the board resets when the motors engage, that reset *is* the measurement.

### Approach profile (the carrot)

How the bot travels to its position target — home after a push, or a new spot
you commanded. Position hold normally chases `enc_pos_target` directly, so the
spring grows with distance: a big error makes it charge, and past ~4 deg of
lean this chassis goes over. With `lead_max > 0` it chases a *carrot* instead:

- the carrot travels toward the target at `return_rate`, speeding up and
  slowing down at `return_accel` so it arrives stopped;
- it is then clamped to within `lead_max` ticks of the wheels, which caps the
  spring at `lead_max / scale_d` degrees of lean however far away the target is.

A push is therefore arrested first (carrot held near the wheels, small spring,
damping does the stopping) and the bot is then walked to the target at a steady
pace. Dashboard move buttons are paced by the same profile. The damping term
subtracts the carrot's achieved speed, so a commanded move is not braked while
it runs, while a push still gets full braking. The carrot restarts at the
wheels whenever the target is re-snapped (arm, disarm, fall, recovery).

Start at `lead_max = 40`, `return_rate = 150`, `return_accel = 100`.
`pos_encError` still shows distance from the target; the correction columns and
the ring's `pos_carrotLead` / `pos_carrotVel` follow the carrot.

### Motion gate

`motor_hal_set_both()` is the only function that sends a non-zero drive command
to the RoboClaw, and it asks a gate registered by `robot.c` immediately before
every send: armed, trying, e-stop not latched, IMU has produced 50 samples, IMU
not stale, `|theta - trim| <= oob_angle_deg`. Any failure sends **coast** (duty
0) instead — not "nothing", which in velocity mode leaves the controller holding
its last speed. Default is deny until the control loop registers the gate. Every
change is logged at WARN: `motion gate: OPEN` / `motion gate: CLOSED -- <reason>`.

### Arm at boot

With `arm_at_boot = 1` the bot sits inert on its kickstand and arms itself when
stood up and held still. The IMU's fused angle starts far off after boot and
slews to the true angle over ~10 s; parked on the side where that slew passes
through 0 it used to arm on the kickstand and drive. So both gates now require
the *estimate* to be still, not just the gyro: "kickstand seen" needs out of
bounds with theta steady within 1 deg for 1 s, and arming needs theta steady
within 1 deg through the 250 ms hold. Rejections log
`arm_at_boot: theta slewing ... NOT arming`.

### Backing it up

`robot.conf` exists only on the SD card and is excluded from rsync, so a sync
cannot overwrite a live calibration. That also means nothing backs it up:

```bash
make save-config     # on the bot: copies robot.conf into config/machine/
                     # then commit config/machine/
make install-config  # on a fresh board: restores it, never overwrites
```

---

## Stand-Up Kick

Parked on its kickstand the robot rests at 17–20°, well outside
`oob_angle_deg`, so every actuation path is dead and it cannot be stood up
without picking it up. The kick drives both wheels briefly, in one direction,
to pop it upright — once it crosses back inside `oob_angle_deg` the balance
loop catches it and the kick ends.

Two ways to ask for it, both landing in the same firmware path:

- **Transmitter:** push the drive stick past 25% while it is out of bounds.
  Stick direction sets the kick direction.
- **Dashboard:** hold ⇤ KICK or KICK ⇥ in the action bar.

### This is the only thing allowed to turn the wheels outside the cutoff

So it is fenced in, in `kick_direction()` (`robot.c`) — not in the callers, so
both paths get identical treatment and neither can bypass any of it:

| Guard | Why |
|---|---|
| `armed`, IMU fresh | same preconditions as any other motion |
| `oob_angle_deg < angle ≤ kick_max_deg` | inside, the balance loop owns the wheels; above, it is lying down and a kick only throws it |
| stick/button seen released before each kick | a failsafe frame, a stale packet or a ratcheted throttle resting at its stop cannot hold the wheels on |
| `kick_timeout_ms` per kick | then it refuses until released — a stuck stick cannot spin the wheels indefinitely |
| dashboard request expires after 300 ms | the page repeats the level while held, so a dropped socket or a closed lid stops the wheels rather than latching them |
| goes through `motor_hal_set_both()` | the motion gate still gets its say |

`kick_assist = 0` disables the whole path.

Start with `kick_duty` low and raise it — too little and it rocks without
coming up, too much and it overshoots through vertical and goes over the other
way. The angle it rests at on the kickstand should sit comfortably inside
`kick_max_deg`; if it rests steeper than that, the kick will refuse rather than
attempt a throw from a bad angle.

---

## Yaw Derivative Source

The steering loop closes on `phi_diff = (phi_right - phi_left) / 2`, in degrees
of wheel rotation. That signal has exactly one resolution:

```
360 / ENCODER_TICKS_PER_REV / 2  =  360 / 145.1 / 2  =  1.2405 deg per count
```

Every value it can ever take is a multiple of 1.2405. It is a staircase, and
numerically differentiating a staircase gives an impulse per step, not a rate.
At `yaw kd = 0.0005` and 100 Hz one count produces

```
0.0005 * 1.2405 / 0.01  =  0.060  of duty differential
```

while the proportional response to the same count is `0.005 * 1.2405 = 0.0062`.
The derivative kicks ten times harder than the proportional term, off a signal
that carries no rate information at all.

Measured, 2026-09-20 (`bbot_ring_1789927479098`, 65 s parked and upright,
heading wandering +/-5 counts):

| | |
|---|---|
| yaw `d_term` rms | 0.0338 |
| yaw `p_term` rms | 0.0081 |
| yaw output rms | 0.0363 (i.e. almost entirely D) |
| output sign reversals | 25 / s |
| `mot_dutyDiff` rms | 7% |
| wheel travel | 1786 counts, to net out 5 |

That is a quantisation limit cycle: one count -> D impulse -> duty differential
-> the wheels move -> the next count. It is what the idle yaw jitter has always
been, and it is worse on the kickstand because the wheels are unloaded.

The IMU measures chassis yaw rate directly (gyro Z, `state.psi_dot`), smoothly
and with no staircase. `yaw_gyro_scale` converts it into `phi_diff` units so
`kd` keeps its meaning:

```
phi_diff_rate [deg/s] = psi_dot [deg/s] * yaw_gyro_scale
yaw_gyro_scale        ~ track_width_mm / WHEEL_DIAMETER_MM     (magnitude)
```

**`yaw_gyro_scale = 0` keeps the old encoder derivative**, which is the default
— the sign depends on how the IMU is mounted, so it is measured, not assumed.

### Calibrating it

The ring logs `yaw_psiDot` (raw gyro Z, deg/s) and `yaw_encRate` (the
differenced staircase) every tick, so one ordinary run gives both sign and
magnitude. The scale does **not** have to be set first — `yaw_psiDot` is the
unscaled sensor, so it is populated even while `yaw_gyro_scale` is 0.

1. Kickstand. Turn the bot left and right by hand for ~20 s, at any point
   during a normal capture.
2. Dump the ring and run `python3 tools/yaw_gyro_scale.py <ring.csv>`.
3. It fits `yaw_encRate = k * yaw_psiDot` through the origin over the whole run
   and prints `k`, sign included. A negative `k` means the IMU is mounted so
   +gyro Z is −phi_diff; that is expected, not a mistake. It refuses to give a
   number if the correlation is weak, which would mean gyro Z is not the axis
   the wheels turn about.
4. Enter the value at Tuning → steering → **D src**. The dashboard shows both
   rates live underneath the box; they should now trace the same curve.

Sanity check before trusting it: with the wheels off the ground and the loop
enabled, idle `mot_dutyDiff` should collapse from ~7% rms to near zero, and the
yaw output should stop reversing sign tens of times a second.

Even with this fixed, heading itself is still only known to 1.2405 deg — the
loop cannot hold tighter than about +/-0.6 deg. The gyro fixes the *damping*,
not the *resolution*.

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
| Motor config | `{"type":"set_motor_config","mode":0,"max_amps":2.5}` — `max_amps` >30 is rejected, not clamped |
| Clear current peaks | `{"type":"reset_amp_peaks"}` |
| RC mapping | `{"type":"set_sbus_config","drive_channel":3,"drive_scale":0.25}` |
| Nudge pose / position | `{"type":"nudge","axis":"pose","delta":0.1}` — axes: `pitch` (trim), `pose` (lean), `yaw`, `fwd` |
| Telemetry options | `{"type":"set_telemetry","encoders":true,"pid_states":true}` |
| Arm at boot | `{"type":"set_arm_at_boot","value":true}` — persists to `robot.conf` immediately |

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

[PolyForm Noncommercial License 1.0.0](https://polyformproject.org/licenses/noncommercial/1.0.0/) — free to use, study,
modify and share for any noncommercial purpose. Commercial use requires a
separate license; contact ryan.lush@gmail.com. Full text in
[`LICENSE`](LICENSE).

Third-party code keeps its own license: `src/roboclaw.c` / `include/roboclaw.h`
(Mozilla Public License 2.0, Bartosz Meglicki) and `include/dmpKey.h` /
`include/dmpmap.h` (InvenSense Corporation). `src/dmp_firmware.c` /
`include/dmp_firmware.h` are a generated InvenSense DMP firmware blob, not
original source, and are noted as such in the files themselves.
