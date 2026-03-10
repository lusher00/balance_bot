# Balance Bot - Complete Setup Guide

This guide walks you through setting up the entire system from scratch.

**Estimated time:** 1-2 hours

---

## Hardware Required

- **BeagleBone Blue** — Main robot controller
- **iPhone** — Control interface (iOS 17+)
- **FrSky R-XSR receiver** (optional) — SBUS RC input
- **Jumper T16 or compatible transmitter** (optional) — RC control
- **NPN transistor (2N3904) + 10kΩ resistor** — SBUS signal inverter
- **Xbox controller** (optional) — Manual control via USB
- **Raspberry Pi 5 with vision system** (optional) — Object tracking input over UART
- **2-wheel balancing robot chassis** — With DC motors and quadrature encoders
- **2S–3S LiPo battery** — For BeagleBone and motors

---

## Part 1: BeagleBone Setup (20 minutes)

### 1.1 Install librobotcontrol

```bash
sudo apt update
sudo apt install librobotcontrol librobotcontrol-dev
```

**Verify:**
```bash
rc_version
```

### 1.2 Install Node.js

```bash
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

node --version   # Should be v18+
npm --version
```

### 1.3 Clone balance_bot

```bash
git clone https://github.com/yourusername/balance_bot.git
cd balance_bot
```

### 1.4 Build

```bash
make

# Should output:
# ✅ Build complete: bin/balance_bot
```

### 1.5 Test (Without iPhone)

```bash
sudo ./bin/balance_bot -i none

# Should output:
# ╔══════════════════════════════════════╗
# ║         balance_bot v1.0             ║
# ╚══════════════════════════════════════╝
# [INFO] IMU initialized
# [INFO] Motors initialized
# System ready! Press MODE button to arm.
```

Press **Ctrl+C** to exit.

---

## Part 2: Node.js WebSocket Bridge (10 minutes)

### 2.1 Install Dependencies

```bash
cd balance_bot/server
npm install
```

### 2.2 Test Server

```bash
node server.js

# Should output:
# WebSocket server running on port 8675
# Waiting for balance_bot socket...
```

Keep this running in a second terminal, or use systemd (see Part 5).

---

## Part 3: Hardware Connections (15 minutes)

### 3.1 SBUS Wiring (if using RC receiver)

The R-XSR outputs inverted SBUS (active-low). A single NPN transistor re-inverts the signal before the BeagleBone UART RX pin.

```
R-XSR SBUS out → 2N3904 base (via 10kΩ)
2N3904 collector → BeagleBone P9.26 (UART1 RX) with 10kΩ pull-up to 3.3V
2N3904 emitter  → GND
R-XSR 5V / GND  → BeagleBone 5V / GND
```

**Verify SBUS is being received:**
```bash
sudo ./bin/balance_bot -i sbus -d sbus
# SBUS block should show live channel values
```

### 3.2 Object Tracking Input (optional — RPi5 or similar)

Connect vision system UART TX to BeagleBone UART1 RX (P9.26):

```
Vision system TX → BeagleBone P9.26 (UART1 RX)
GND              → GND
```

Both devices are 3.3V — safe to connect directly (no inverter needed for UART).

The robot expects newline-delimited position frames:
```
CAT,0.30,-0.15,0.87
```

### 3.3 Xbox Controller (optional)

Plug into BeagleBone USB and verify:
```bash
ls /dev/input/js*
# Should show: /dev/input/js0
```

---

## Part 4: iPhone App Setup (20 minutes)

### 4.1 Open Project

```
Xcode → Open → BBotTuneHUD.xcodeproj
```

### 4.2 Configure Signing

1. Select project in navigator
2. Select target **BBotTuneHUD**
3. Signing & Capabilities → select your Apple ID team
4. Set bundle identifier: `com.yourname.BBotTuneHUD`

### 4.3 Build and Install

1. Connect iPhone via USB
2. Select iPhone as target device
3. **⌘R** to build and run
4. On iPhone: trust the developer if prompted
   - Settings → General → VPN & Device Management → trust your Apple ID

---

## Part 5: Network Configuration (5 minutes)

All devices must be on the same WiFi network.

### 5.1 Find BeagleBone IP

```bash
hostname -I
# e.g. 192.168.1.100
```

### 5.2 Test WebSocket Bridge

From a browser or curl on your Mac:
```bash
curl http://192.168.1.100:8675
# Should respond (even if not a valid WebSocket upgrade)
```

### 5.3 Configure iPhone App

1. Open **BBotTuneHUD**
2. Tap **Settings** tab
3. Enter BeagleBone IP: `192.168.1.100`
4. Tap **Reconnect**

You should see:
- ✅ Connected status
- ✅ Live telemetry (battery, loop Hz, armed state)

---

## Part 6: Systemd Auto-Start (optional)

### 6.1 Install Service Units

```bash
sudo cp systemd/balance_bot.service /etc/systemd/system/
sudo cp systemd/balance_bot_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable balance_bot balance_bot_server
```

### 6.2 Start Services

```bash
sudo systemctl start balance_bot
sudo systemctl start balance_bot_server
```

### 6.3 Verify

```bash
systemctl status balance_bot
systemctl status balance_bot_server
```

### 6.4 View Logs

```bash
journalctl -u balance_bot -f
journalctl -u balance_bot_server -f
```

---

## Part 7: First Balance Test (30 minutes)

### 7.1 Start Services

**If not using systemd:**

```bash
# Terminal 1
sudo ./bin/balance_bot -i sbus    # or -i none, -i xbox

# Terminal 2
cd server && node server.js
```

**If using systemd:**
```bash
sudo systemctl start balance_bot balance_bot_server
```

### 7.2 IMU Zero

1. Place robot upright on flat ground
2. Open **BBotTuneHUD** → **IMU** tab
3. Tap **Zero display at current angles**
4. 3D cube should sit level

### 7.3 Arm

**Via RC transmitter:**
- Kill switch (CH6/SB) → fully high
- Arm switch (CH5/SA) → fully high

**Via iPhone app (no controller needed):**
- Control tab → tap **ARM**
- Rejected if lean angle > ~14°

Green LED on BeagleBone confirms armed state.

### 7.4 Tune Balance PID

Start with conservative gains and work up:

| Gain | Starting value | Direction |
|------|---------------|-----------|
| Kp | 30.0 | Increase until it oscillates, then back off |
| Kd | 4.0 | Increase to damp oscillation |
| Ki | 0.0 | Add small amount last to remove steady-state error |

1. **PID Tuning** tab → select **D1: Balance**
2. Adjust sliders → tap **Send**
3. Changes take effect immediately — no restart needed
4. Watch live response in telemetry

### 7.5 Add Steering

Once balance is stable:
1. Enable **D3: Steering** in PID Tuning tab
2. Use RC transmitter right stick or iPhone to command turns

---

## Troubleshooting

### "Failed to initialize IMU"

```bash
# Calibrate accelerometer (run once on flat surface)
rc_calibrate_accel

# Calibrate gyro
rc_calibrate_gyro

# Check I2C bus
sudo i2cdetect -r -y 2
# Should show device at 0x68
```

### "Motor init failed"

```bash
# Check PRU firmware
ls /lib/firmware/am335x-pru*

# Test motors manually
rc_test_motors
```

### "No telemetry in iPhone app"

```bash
# Check socket exists
ls -la /tmp/balance_bot.sock

# Check Node.js bridge is running
systemctl status balance_bot_server

# Run bridge manually to see errors
cd balance_bot/server && node server.js
```

### "SBUS not receiving"

```bash
# Test SBUS live
sudo ./bin/balance_bot -i sbus -d sbus

# Check UART device
ls /dev/ttyO*

# Verify baud rate — SBUS requires 115200 8E2 inverted
# Confirm inverter circuit wiring (2N3904 + 10kΩ pull-up)
```

### "Robot won't balance"

1. **Motors backwards** — swap motor wires or negate duty in code
2. **IMU orientation wrong:**
   ```bash
   rc_test_dmp
   # Tilt robot forward — theta should increase positively
   ```
3. **Kp too low** — increase in steps of 5
4. **Encoders not working:**
   ```bash
   rc_test_encoders_eqep
   # Rotate wheels — verify counts change
   ```

### Useful Test Commands

```bash
rc_test_dmp              # IMU angles live
rc_test_motors           # Drive motors
rc_test_encoders_eqep    # Encoder counts
rc_test_buttons          # MODE / PAUSE buttons
rc_test_adc              # Battery voltage
```

---

## Setup Checklist

- [ ] librobotcontrol installed and `rc_version` works
- [ ] Node.js v18+ installed
- [ ] `make` succeeds
- [ ] `balance_bot -i none` runs without errors
- [ ] Node.js bridge connects to socket
- [ ] BBotTuneHUD connects and shows telemetry
- [ ] IMU angles visible in 3D tab
- [ ] Robot arms via app or RC transmitter
- [ ] Robot attempts to balance
- [ ] PID changes from app take effect live
- [ ] SBUS channels visible (if using RC)
- [ ] Systemd services start on boot (if configured)

---

## You're Done! 🎉

Your balance_bot is running with full iPhone telemetry and live PID tuning.

**Tip:** Save your tuned PID values to `pidconfig.txt` so they persist across reboots.