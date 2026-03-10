# Balance Bot - Complete Setup Guide

This guide walks you through setting up the entire system from scratch.

**Estimated time:** 2-3 hours

---

## Hardware Required

- **BeagleBone Blue** - Main robot controller
- **Raspberry Pi 5** - Vision system (already set up with cat_track)
- **iPhone** - Control interface (iOS 14+)
- **Xbox Controller** (optional) - Manual control
- **2-wheel balancing robot chassis** - With motors and encoders
- **Jumper wires** - For UART connection
- **Battery** - For BeagleBone (2S-3S LiPo recommended)

---

## Part 1: BeagleBone Setup (30 minutes)

### 1.1 Install librobotcontrol

If not already installed:

```bash
cd ~
git clone https://github.com/beagleboard/librobotcontrol.git
cd librobotcontrol/library
make
sudo make install
sudo ldconfig
```

**Verify:**
```bash
rc_version
# Should print version info
```

### 1.2 Install Node.js

```bash
# Install Node.js 14+ (if not present)
curl -fsSL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get install -y nodejs

# Verify
node --version  # Should be v14+
npm --version
```

### 1.3 Copy balance_bot Code

```bash
# Create directory in librobotcontrol
cd ~/librobotcontrol
mkdir -p balance_bot
cd balance_bot

# Copy all files from this package:
# - include/
# - src/
# - Makefile
# - README.md

# Or clone from GitHub (once pushed):
# git clone https://github.com/yourusername/balance_bot.git
```

### 1.4 Build balance_bot

```bash
cd ~/librobotcontrol/balance_bot
make

# You should see:
# ✅ Build complete: bin/balance_bot
```

### 1.5 Test (Without iPhone)

```bash
sudo ./bin/balance_bot

# You should see:
# [INFO] Initializing balance_bot
# [INFO] IMU initialized
# [INFO] Motors initialized
# ...
# Ready! Press MODE to arm.
```

Press **Ctrl+C** to exit.

---

## Part 2: Node.js Server Setup (15 minutes)

### 2.1 Create Server Directory

```bash
cd ~
mkdir cat-follower-server
cd cat-follower-server
```

### 2.2 Initialize npm

```bash
npm init -y
```

### 2.3 Install Dependencies

```bash
npm install express ws
```

### 2.4 Create server.js

```bash
nano server.js
```

Paste the Node.js server code (provided separately in `server.js`).

Save and exit (Ctrl+X, Y, Enter).

### 2.5 Test Server

```bash
node server.js

# Should output:
# Cat Follower Server running on port 8080
# IPC connected to balance_bot
```

Keep this running in a separate terminal or use `pm2`:

```bash
# Optional: Install pm2 for auto-restart
sudo npm install -g pm2
pm2 start server.js --name cat-follower
pm2 save
pm2 startup
```

---

## Part 3: Hardware Connections (15 minutes)

### 3.1 UART Connection (RPi5 ↔ BeagleBone)

Connect with jumper wires:

```
RPi5 Header          BeagleBone Blue
-----------          ---------------
Pin 8  (GPIO14 TX) → UART1 RX (Pin P9.26)
Pin 10 (GPIO15 RX) → UART1 TX (Pin P9.24) [optional]
Pin 6  (GND)       → GND
```

**Important:** Both devices are 3.3V - safe to connect directly.

### 3.2 Verify UART

On BeagleBone:
```bash
# Should see cat data streaming
sudo cat /dev/ttyS1

# Output should look like:
# CAT,0.30,-0.15,0.87
# CAT,0.31,-0.14,0.88
# ...
```

If you see nothing, check:
- Wiring
- cat_track is running on RPi5
- Correct UART pins

### 3.3 Xbox Controller (Optional)

```bash
# Plug Xbox controller into BeagleBone USB

# Verify device node
ls /dev/input/js*
# Should show: /dev/input/js0
```

---

## Part 4: iPhone App Setup (30 minutes)

### 4.1 Install Xcode

On your Mac:
1. Install Xcode from App Store (if not installed)
2. Open Xcode
3. Install command line tools if prompted

### 4.2 Open Project

1. Unzip `CatFollowerApp.zip` (provided separately)
2. Double-click `CatFollowerApp.xcodeproj`
3. Wait for dependencies to load

### 4.3 Configure Signing

1. Select project in left sidebar
2. Select target "CatFollowerApp"
3. Go to "Signing & Capabilities"
4. Select your Apple ID team
5. Change bundle identifier if needed (e.g., `com.yourname.catfollower`)

### 4.4 Build and Install

1. Connect iPhone via USB
2. Select iPhone as target device (top toolbar)
3. Click ▶️ Build and Run
4. On iPhone: Trust the developer (Settings → General → Device Management)
5. App should launch

---

## Part 5: Network Configuration (10 minutes)

All devices must be on the same WiFi network.

### 5.1 Find IP Addresses

**BeagleBone:**
```bash
hostname -I
# Note the IP, e.g., 192.168.1.100
```

**RPi5:**
```bash
hostname -I  
# Note the IP, e.g., 192.168.1.139
```

### 5.2 Test Connectivity

From iPhone, open Safari and navigate to:
- `http://192.168.1.139:5000` - Should show cat_track video
- `http://192.168.1.100:8080` - Should show Node.js server

If these don't work:
- Check WiFi connection
- Check firewall settings
- Verify services are running

### 5.3 Configure iPhone App

1. Open CatFollowerApp
2. Tap Settings (gear icon)
3. Enter BeagleBone IP: `192.168.1.100`
4. Enter RPi5 IP: `192.168.1.139`
5. Tap "Connect"

You should see:
- ✅ Connected status
- Live video stream
- Real-time telemetry

---

## Part 6: First Run (30 minutes)

### 6.1 Start All Services

**Terminal 1 - balance_bot:**
```bash
cd ~/librobotcontrol/balance_bot
sudo ./bin/balance_bot
```

**Terminal 2 - Node.js server:**
```bash
cd ~/cat-follower-server
node server.js
# OR if using pm2:
pm2 start server.js
```

**RPi5 - cat_track (should already be running):**
```bash
# Verify it's running
ps aux | grep cat_track
```

**iPhone:**
- Launch CatFollowerApp
- Connect to BeagleBone

### 6.2 Initial Balance Test

**SAFETY FIRST:**
- Place robot on soft surface
- Keep hands clear of wheels
- Have MODE button accessible

**Steps:**

1. **Check robot is upright** - Hold it vertically
2. **iPhone app:**
   - Go to PID Tuning screen
   - Verify D1_balance is enabled
   - Verify D2_drive is disabled
   - Verify D3_steering is disabled
3. **Press MODE button** on BeagleBone to ARM
   - Green LED should light
   - iPhone should show "ARMED"
4. **Gently release robot**
   - It should attempt to balance
   - If it falls, press MODE to disarm

### 6.3 Tuning Balance

If robot falls or oscillates badly:

1. **Reduce Kp:**
   - iPhone app → PID Tuning
   - D1_balance
   - Slide Kp down to 30
   - Try again

2. **Adjust Kd:**
   - If oscillating: Increase Kd to 6-7
   - If sluggish: Decrease Kd to 3-4

3. **Iterate:**
   - Small changes
   - Test each time
   - Use iPhone app's real-time graph to see response

### 6.4 Add Steering

Once balance is stable:

1. Enable D3_steering in iPhone app
2. Arm robot
3. Use Xbox controller or iPhone app to command small turns
4. Tune steering PID if needed

### 6.5 Test Cat Following

1. Enable "Cat Follow" mode in iPhone app
2. Place cat in view of RPi5 camera
3. Robot should track cat's position
4. iPhone app should show cat overlay on video

---

## Part 7: Auto-Start on Boot (Optional)

### 7.1 Create systemd Service

```bash
sudo nano /etc/systemd/system/balance_bot.service
```

Paste:
```ini
[Unit]
Description=Balance Bot Control System
After=network.target

[Service]
Type=simple
User=debian
WorkingDirectory=/home/debian/librobotcontrol/balance_bot
ExecStart=/home/debian/librobotcontrol/balance_bot/bin/balance_bot
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable balance_bot
sudo systemctl start balance_bot
```

### 7.2 Auto-Start Node.js

```bash
cd ~/cat-follower-server
pm2 start server.js
pm2 save
pm2 startup  # Follow instructions
```

---

## Troubleshooting

### "Failed to initialize IMU"

**Causes:**
- IMU not calibrated
- I2C bus conflict

**Solutions:**
```bash
# Calibrate IMU
rc_calibrate_gyro

# Check I2C
sudo i2cdetect -r -y 2
# Should show device at 0x68
```

### "Motor init failed"

**Causes:**
- PRU firmware not loaded
- Motor driver fault

**Solutions:**
```bash
# Check PRU firmware
ls /lib/firmware/am335x-pru*

# Test motors manually
rc_test_motors
```

### "No telemetry in iPhone app"

**Causes:**
- Node.js server not running
- balance_bot not creating socket
- Firewall blocking

**Solutions:**
```bash
# Check socket exists
ls -la /tmp/balance_bot.sock

# Check Node.js is running
ps aux | grep node

# Check for errors in Node.js
node server.js
# Look for connection errors
```

### "Video not showing in iPhone app"

**Causes:**
- cat_track not running
- Wrong RPi5 IP address
- Network connectivity

**Solutions:**
```bash
# Test video directly in browser
# On Mac: open http://192.168.1.139:5000

# Check cat_track is running on RPi5
ssh rp5@192.168.1.139
ps aux | grep cat_track
```

### "Robot won't balance"

**Common issues:**

1. **Motors backwards:**
   - Swap motor wires
   - Or invert in code

2. **IMU orientation wrong:**
   ```bash
   # Test IMU orientation
   rc_test_dmp
   # Tilt robot, verify angles match
   ```

3. **Kp too low:**
   - Increase gradually until it balances

4. **Encoders not working:**
   ```bash
   rc_test_encoders_eqep
   # Rotate wheels, verify counts change
   ```

---

## Next Steps

Once basic balance works:

1. **Tune PID gains** for smooth performance
2. **Test cat following** mode
3. **Try Xbox controller** manual control
4. **Experiment with debug features** in iPhone app
5. **Save tuned PID values** to pidconfig.txt

---

## Getting Help

**Logs to check:**
```bash
# balance_bot output
sudo ./bin/balance_bot

# Node.js server
node server.js  # Run in foreground to see logs

# System logs
journalctl -u balance_bot -f
```

**Useful test commands:**
```bash
rc_test_dmp          # Test IMU
rc_test_motors       # Test motors
rc_test_encoders_eqep # Test encoders
rc_test_buttons      # Test buttons
rc_test_dsm          # Test Xbox/DSM input
```

---

## Success Checklist

- [ ] librobotcontrol installed
- [ ] Node.js installed
- [ ] balance_bot compiles
- [ ] Node.js server runs
- [ ] UART connected and receiving cat data
- [ ] iPhone app installed and connects
- [ ] Video stream visible in iPhone app
- [ ] Telemetry updates in real-time
- [ ] Robot balances when armed
- [ ] PID tuning works from iPhone
- [ ] Cat following works (if using RPi5)
- [ ] Xbox controller works (if connected)

---

## You're Done! 🎉

Your balance_bot system is now fully operational with iPhone app control!

Enjoy your self-balancing, cat-following robot!
