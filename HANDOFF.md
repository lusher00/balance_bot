# Robot accessory stack handoff — 2026-09-21

This describes the **repository state** at the commits listed below and the
last hardware observations reported in the conversation. A green service on a
board after these commits has not been independently checked here.

| Repository | Reviewed main | Responsibility |
| --- | --- | --- |
| `batt_monitor` | `d704e69` | BeagleBone AIN5 voltage, policy, shutdown event |
| `robot-link` | `6793154` | Bone/Pi USB transport, event forwarding, Pi audio/shutdown |
| `oled-utils` | `d0e52c5` | Pi display, link state, battery, two IP addresses |
| `balance_bot` | `b5817c5` | Motion controller; vendored Bone OLED copy |
| `hailo-tracker` | `bfdcb5f` | Pi camera and Hailo inference, HTTP port 8080 |

## What is working or implemented

- Bone USB gadget is `192.168.7.2/24`; Pi USB Ethernet is
  `192.168.7.1/24`. Both directions were pingable. `robot-link-boned` binds
  only to the Bone USB address and now waits for it at boot. The Pi reconnects
  and the heartbeat ends a dead session.
- The Bone had reported a connected Robot Link session and a battery voltage.
  The Pi had reported a connected session. `batt_monitor` publishes atomic
  `/run/batt_status.json`; Robot Link forwards voltage to the Pi status file.
- `batt_monitor` now defaults to AIN5 and includes `--calibrate` again. The
  installed setup used divider `11.2702`, three cells, 10-second sampling,
  three critical readings, a two-hour trend qualifier, and 15 seconds of Bone
  shutdown grace. A reboot showed the three-minute timer counting down.
- In a prior dry run, the Bone requested Pi shutdown at `9.510 V`; the Pi
  accepted it and logged `DRY RUN: systemctl poweroff`. This proves the
  request path under dry run, not a real powered-off Pi.
- OLED code includes real Pi/Bone session state, Bone voltage, and both Pi IP
  addresses; the Pi panel was observed lit after manual unit fixes. The Bone
  OLED code is vendored into `balance_bot` at the reviewed revision.
- `hailo-tracker` serves HTTP on port `8080` when running. After a Pi kernel
  update to `6.8.0-1064-raspi`, PCIe still enumerated Hailo-8, but
  `/dev/hailo0` was absent: DKMS had installed `hailo_pci/4.23.0` only for
  `1053` and `1057`. Rebuild for the running kernel after installing matching
  headers. The conversation did not include a post-rebuild service check.

## Shutdown behavior and recovery

For a 3S pack the current defaults are warning `10.5 V`, low `9.9 V`, and
critical `9.6 V`. `batt_monitor` alone decides shutdown. It normally requires
three critical samples; one critical sample qualifies only if the watcher has
seen two continuous hours below warning and at least `0.30 V` net drop. The
watcher's history is in memory and starts over on restart. It writes a unique
shutdown event, gives the Pi 15 seconds, then requests Bone shutdown. Robot
Link sends that event over USB with a configured Pi shutdown delay of 5
seconds. The RoboClaw has separate internal low-voltage protection. The Bone
cannot electrically switch off the Pi supply.

Use `sudo ~/batt_monitor/setup-bone.sh install` on the Bone. The early
`batt_check` publishes a voltage without shutdown; `batt_monitor.timer` waits
three minutes after boot before starting the watcher. `batt_monitor.service`
being `static` is expected. A timer first enabled after three minutes of
uptime may fire immediately.

```bash
sudo ~/batt_monitor/setup-bone.sh status
systemctl list-timers batt_monitor.timer --all --no-pager
sudo ~/batt_monitor/setup-bone.sh disable
sudo ~/batt_monitor/setup-bone.sh enable
```

`disable` stops the timer and watcher while keeping the boot check. `enable`
restores the timer and starts the watcher now. The installer saves the previous
config as `/etc/default/batt_monitor.previous`, but every `install` writes its
hardcoded AIN5 settings. Do not use `sudo make install` for this deployment:
its current Makefile enables the watch service directly, bypassing the timer.

## Remaining work

1. **Unify battery installers.** Make `make install` honor the three-minute
   timer, or have it delegate to `setup-bone.sh`. Preserve user calibration
   across reinstalls rather than silently replacing `/etc/default/batt_monitor`.
   Check `batt_check` uses the same AIN5 calibration as the watcher.
2. **Verify a real shutdown on a controlled bench setup.** Confirm the Pi
   receives the committed event with dry run off, halts cleanly before the
   Bone, and the RoboClaw response meets the machine safety requirements.
   There is still no electrical Pi power disconnect.
3. **Repair the OLED installer upstream.** Its generated unit still names
   `i2c gpio` and sets `ProtectHome=yes`. The Pi image lacks `gpio`, and the
   code runs under `/home/rp5`. Make group selection conditional and allow
   read access to the home directory, then install and check the physical
   panel. Keep the Bone vendored file synchronized when the upstream code
   changes.
4. **Complete Pi voice output.** Confirm `/usr/local/bin/speak_text` exists,
   routes PCM to `plughw:CARD=MAX98357A,DEV=0`, and is intelligible; the
   earlier service failed with `FileNotFoundError` and no later audible voice
   was confirmed. WAV playback needs a separate audible check.
5. **Confirm Hailo after the kernel update.** For the currently booted kernel,
   check matching headers, `dkms status`, `/dev/hailo0`, `hailortcli scan`,
   `systemctl status hailo-tracker`, and `curl localhost:8080/healthz`. Add a
   stable driver rebuild/install step so future kernel updates do not leave
   the tracker restart looping.
6. **Future additions:** Lidar input and SLAM on the Pi, then clearly scoped
   high-level results over Robot Link. ROS 2 remains an architectural option,
   not part of the running transport or Bone control loop. The design lives
   in [PROTOCOL_DESIGN.md](PROTOCOL_DESIGN.md).

## Quick status commands

On the Bone:

```bash
sudo ~/batt_monitor/setup-bone.sh status
sudo robot-linkctl status
systemctl status robot-link-boned batt_check batt_monitor.timer batt_monitor --no-pager
sudo cat /run/batt_status.json
```

On the Pi:

```bash
systemctl status robot-linkd oled-status hailo-tracker --no-pager
sudo cat /run/robot-link/status.json
ls -l /dev/hailo0
curl -fsS http://127.0.0.1:8080/healthz
```
