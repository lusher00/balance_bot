# balance_bot — handoff

**Written:** 2026-08-23
**Board:** BeagleBone Blue, single-core 1 GHz Cortex-A8, Debian Trixie, kernel `6.18.34-bone37`
**Host:** `boneblue-0` (user `debian`). Repo lives on the Mac at `~/balance_bot`, deployed by a VS Code task (`rsync` → `make` → `make install`).

This replaces the 2026-08-17 handoff, which was written around the telemetry
problem. That problem is solved. The document is now organised around what is
still open.

---

## 0. Read this before doing anything

**Standing constraints from Ryan. These are not suggestions.**

1. **Do not commit to git.** Make the file changes; committing is his call.
   Git in the assistant's environment also has no `user.name`/`user.email`.
2. **Do not diagnose the read-only fault as failing storage media.** Both an SD
   card and eMMC have been run. The storage device has been blamed repeatedly;
   swapping it has never fixed it. There are months of data on this. Any theory
   that reduces to "the card is bad" has already been tested and disproved by
   the hardware swap.
3. **Wifi on this box is managed by `iwd` (`iwctl`).** Not NetworkManager, not
   connman, not wpa_supplicant. `nmcli` does not exist here.
4. **UI work goes slowly, one item at a time**, from `web/UI_BACKLOG.md`.
5. Deploy is his VS Code task. Do not hand him build instructions unless he
   asks — he syncs, builds and restarts from the editor.

**Environment gotchas that have each cost real time:**

- `/usr/sbin` is not on the `debian` user's `PATH`. `iw: command not found` and
  `swapon: command not found` both meant "not on your PATH", not "not
  installed". Use full paths in anything you hand him.
- **zsh on macOS does not treat `#` as an interactive comment.** A trailing
  `# explanation` on a pasted command becomes extra arguments. Do not put
  comments on command lines meant for the Mac.
- `make` builds `bin/balance_bot`; the unit runs `/usr/local/bin/balance_bot`.
  `make install` is required — `make` alone plus a restart runs the old binary.
- `systemctl reset-failed bbb_oled` is needed after `StartLimitBurst` trips, or
  a plain restart looks like the fix did not work.

---

## 0.5 STOP — the U-Boot capture moves the whole fault (2026-08-23, serial console)

**Read this before §1. Most of what follows it was reasoning about the wrong
layer.** A serial-console capture of a failed boot shows this, from U-Boot
2019.04, on a cold start:

```
Reset Source: Power-on reset has occurred.
MMC:   OMAP SD/MMC: 0, OMAP SD/MMC: 1
Loading Environment from EXT4... Card did not respond to voltage select!
...
Card did not respond to voltage select!      (repeatedly)
switch to partitions #0, OK
mmc1(part 0) is current device
** No partition table - mmc 1 **
```

`Card did not respond to voltage select` is the SD card failing **CMD8 /
ACMD41 voltage negotiation** — the very first conversation the host has with
the card, before any filesystem, any driver, any operating system.

**What this eliminates, all at once:**

**CORRECTION (2026-08-24, later).** The table below overreached, and the row
about EDMA is wrong. What the U-Boot capture actually proves is that **a card
which has already failed stays failed across a power cycle** — it does not
prove the *initiating* event happens without Linux. A card latched into an
inoperable state by something at runtime looks exactly like this at the next
power-on. Consequence to keep straight: U-Boot failing is downstream evidence,
not an independent occurrence, and it cannot exonerate anything that ran
before it. Read the row as struck through; see §0.6.

| Suspect | Why it is now dead |
|---|---|
| ~~**EDMA / shared DMA engine (§1.3)**~~ | ~~U-Boot's MMC stack does not use the kernel's EDMA path…~~ **WRONG — see the correction above. This lead is reopened in §0.6.** |
| The 8250 RX-DMA teardown WARN | Also not exonerated. Struck for the same reason. |
| Linux mmc/sdhci driver | Different stack, same failure. |
| Kernel version (4.1x vs 6.x) | Neither is running. |
| Write pressure, journald, apt, config saves | No writes exist at this point in the boot. |

The fault is **electrical, in the card-initialisation path**, and it is present
at power-on. Everything in §1 that reasoned about *when during Linux runtime*
it happened was measuring a downstream symptom.

**What is left, and how to separate it.** Voltage select depends on VDD being
present and stable at the socket and on CMD/CLK integrity. So: the card, the
socket (contacts, solder joints, retention), or the 3.3 V feed to the slot.
Ryan's standing data — swapping media has never fixed it, across both an SD
card and eMMC — does not become wrong here; it *points*. If the medium is not
the variable and the failure is at power-on voltage negotiation, **the socket
and its supply are the leading candidates**, not the card in it.

The cheap decisive test does not require blaming anything: **read the card in a
USB reader on the Mac.** If it mounts and reads clean, the card is exonerated
and the board-side hardware is what is left. That is a test that can only
narrow things.

**Second finding from the same capture: there is no fallback boot device.**
`** No partition table - mmc 1 **` — the eMMC's partition table is gone
(consistent with the `dd if=/dev/zero of=/dev/mmcblk1 bs=1M count=10` in shell
history, and with the kernel never printing `mmcblk1: p1 p2 p3`). So a card
that fails to init is a board that cannot boot at all; U-Boot falls through to
PXE over USB RNDIS and hangs. Restoring a bootable eMMC turns every future
occurrence from a brick into an inconvenience.

**Console baud trap.** U-Boot talks at **115200**; the kernel command line says
`console=ttyS0,921600n8`. A minicom session left at 115200 shows U-Boot
perfectly and then garbage — or apparent silence — the moment Linux takes over.
Switch to 921600 after the handoff.

---

## 0.6 SBUS — TESTED AND REFUTED (2026-08-24)

**The control run killed this.** Same binary, same bridge, same transmitter
switched on, only `-i none` so balance_bot never opens ttyS5:

```
209.914  bbot_run START id=1787601030 input=none server=1 label='control 1'
424.617  mmc0: tried to HW reset card, got error -110
```

**214.7 s — 3.6 minutes.** The control died *faster* than the sbus run's 13.2
minutes. SBUS traffic is not the variable; it does not cause the fault and it
does not protect against it. Everything below this line is kept only so the
reasoning is on the record — do not act on it.

| condition | lifetime |
|---|---|
| idle, no services at all | 206 min, no failure |
| balance_bot, `-i none` | **3.6 min** |
| balance_bot, `-i sbus` | 13.2 min |

**What survives: the variable is `balance_bot running`, and nothing narrower.**
Two runs minutes apart in board state, differing only in whether a UART was
opened, both dead in minutes; an idle board fine for over three hours.

**Next split — what balance_bot does that an idle board does not:**

1. **ttyS1 to the RoboClaw**, opened at 460800 and polled at loop rate. This is
   the port with 12.3 M interrupts, and the one whose RX DMA teardown throws
   `omap_8250_rx_dma_flush` on every start — in *both* arms above, so it is a
   constant across the experiment and cannot be the discriminator between them,
   but it is very much a candidate for the difference against idle.
2. **The IMU on i2c-2** at 100 Hz.
3. **Writes** — `/tmp/balance_bot.log`, `robot.conf`, now capped and batched.

The cheapest next run is the one already written up in the claw section:
**balance_bot running with the RoboClaw disconnected entirely, power and
serial.** Encoder reads fail harmlessly and log `(-2)`. Survives → the fault is
in the RoboClaw path. Dies in minutes → it is the IMU or the writes, and the
next split is those two.

---

## 0.6-old (superseded) — the SBUS reasoning, kept for the record

Two claims from the bench, both of which beat anything inferred from the logs:

- **It dies within minutes when balance_bot is running.**
- **It does so whether or not the motors are powered.** So not motor current,
  not vibration. Both of those mechanisms are out.

That leaves what balance_bot does that nothing else on the board does: it opens
**two UARTs** and drives them hard. `/dev/ttyS5` for SBUS in, `/dev/ttyS1` to
the RoboClaw at 460800, plus IMU reads on i2c-2. The motors are downstream of
all of it and can be unpowered while every byte still moves.

**And this reconciles the two timed events, which "minutes" otherwise
contradicts:**

| event | balance_bot alive before failure | SBUS RX traffic |
|---|---|---|
| A | 33 min | **none** — unit ran the `-q -i none` fallback, no input source |
| B | 184 min | **none** — transmitter off the whole time, so failsafe, no frames |
| bench, 08-24 | **13.2 min** | **live** — transmitter on, ~143 frames/s into ttyS5 |
| idle, 08-24 | 206 min, no failure | none — no services running at all |

**First properly instrumented run, 2026-08-24** (`start.sh` kmsg stamps):

```
13560.040  bbot_run START id=1787597741 input=sbus
13562.165  omap_8250_rx_dma_flush WARN   (PID 797 python3 = roboclaw_reset.py, ttyS1)
13566.502  balance_bot sets custom speed on ttyS5   <- SBUS opens here
14356.399  mmc0: tried to HW reset card, got error -110
14356.790  bbot_run END elapsed=796s
```

**789.9 s — 13.2 minutes — from ttyS5 opening to mmc0 dying.** The stamps did
their job: no stopwatch, no ambiguity about which condition was running.

The `Segmentation fault` from balance_bot is **downstream, not a cause**: it
lands at the same instant as the remount, and it is the same mechanism that
made `journalctl` segfault in the 08-23 capture — text pages can no longer be
faulted in from a dead card, so the process takes a SIGBUS/SIGSEGV. Do not
chase it.

**What this does and does not establish.** 13 min with live SBUS against 33 min
(`-i none`) and 184 min (transmitter off) is the right direction, but those two
are from earlier days and different board states, and the board has failed
repeatedly since. Against today's only same-day baseline — 206 minutes idle
with *no services at all* — it is suggestive and nothing stronger, because that
baseline does not isolate SBUS from "balance_bot running at all".

**The missing run is `--input none` today**, with balance_bot up. That is the
one comparison that separates the two, and until it exists this table is three
unlike measurements and one good one.

The two long survivals are exactly the two runs with an idle SBUS UART. That is
a real pattern across the only three data points that have a measured
duration, and it was invisible until Ryan said the motors were irrelevant.

So the variable to test is not "balance_bot" — it is **incoming SBUS frames on
ttyS5, and therefore that UART's RX DMA**, which shares the EDMA at 0x49000000
with all three MMC controllers, and whose teardown path already throws
`omap_8250_rx_dma_flush` on this board at every service start.

**The test, and it needs no instrumentation beyond a clock.** balance_bot
running in both cases, motors irrelevant:

1. transmitter **off** (failsafe, no frames arriving) — how long does it live?
2. transmitter **on** (~143 frames/s) — how long does it live?

Hours versus minutes settles it. If it is the serial path, §1.4 step 3 (the DT
overlay that takes ttyS5 off DMA and forces PIO) goes from speculative to the
obvious next move, and `tools/edma_probe.sh --reset N` becomes worth running
properly.

**A caution, stated plainly.** "Powering the transmitter on breaks the board"
is the exact shape of the misattribution that burned a week in §5 — that one
turned out to be 2.4 GHz channel congestion and the transmitter was innocent.
This is a different claim with a different mechanism (RC receiver → SBUS UART →
RX DMA → shared EDMA, no radio involved), and the wifi confound is gone now
that the link is on 5 GHz. But run the comparison twice before believing it,
and prefer `-i none` over "transmitter off" for one of the runs — it removes
the receiver from the picture entirely rather than just making it quiet.

---


### Measuring the Blue's own input rail — AIN5

Noted 2026-08-24, not yet used. **AIN5 reads the DC barrel jack through a
47k/4.7k divider** — the input to the board's own regulator. That is the one
rail nobody has ever measured during a failure: `/run/batt_status.json` is the
*pack*, and it read a healthy 12.445 V at the moment mmc0 died, which says
nothing about what the Blue was actually being fed.

    multiplier    (47 + 4.7) / 4.7   = 11.000
    ADC           1.8 V ref, 12-bit
    resolution    4.835 mV per count at the jack
    full scale    19.80 V  (raw 4095)
    12.00 V       raw 2482

Read it at `/sys/bus/iio/devices/iio:device*/in_voltage5_raw` (the device whose
`name` is the am335x ADC); use `in_voltage_scale` if the kernel exposes one.

Jack spec is **9–18 V**, 12 V typical. The divider's 19.8 V full scale sits
just above that maximum — it was sized for this range, which corroborates the
figure.

If this gets sampled properly, **report the minimum per interval, not the
instantaneous value**. A 10 s poll (all ro_beacon.sh does) sails past a 20 ms
sag without a flicker; a continuous sampler reporting min/mean/max per second
catches anything lasting tens of ms. Sub-microsecond spikes still need a scope
— a sysfs one-shot read is a syscall plus a conversion, so the ceiling is a
few kHz on this CPU.


## 1. The read-only filesystem — primary evidence at last

This is the oldest open fault and, until 2026-08-23, it had **no primary
evidence attached to it at all**. That was not for lack of looking: the failure
destroys the means of observing itself. journald cannot write to a read-only
disk and falls back to volatile storage that dies at reboot; `dmesg` needs a
shell and the shell is gone; `/var/log` is read-only by definition of the
fault. Every previous theory was therefore inference from the wreckage.

Ryan got a live shell on the board during an event and dumped the ring buffer
to `ro_dump.txt` (4813 lines). What follows is from that file.

**What the file actually is.** One boot, 91.8 minutes of uptime, 1677 kernel
records — of which **189 are bbb_oled's probe rejections**, so the real content
is under 1500 lines and most of that is the boot. Anchoring uptime to wall
clock: `batt_monitor` and `bbb_oled` both report starting 09:56, but ext4's own
`last_error_time` (1787497822 = 11:10:22 EDT) pins the read-only remount at
t=3590, which puts boot at **10:10:32 EDT**. The systemd "since" times are
~14 minutes behind because they were recorded before timesyncd corrected the
clock. Durations in `systemctl status` are fine; absolute times in this dump
are not, and correlating the journal against dmesg without accounting for that
will produce a phantom.

### 1.1 The failure itself

Clean boot. mmc0 (the SD card) enumerates normally at t=4.4 s. **No mmc message
of any kind for the next 59 minutes.** Then:

```
[3585.256] mmc0: tried to HW reset card, got error -110
[3585.257] mmcblk0: recovery failed!
[3585.257] I/O error, dev mmcblk0, sector 3874952 op 0x1:(WRITE) flags 0x9800 phys_seg 3
[3585.259] Aborting journal on device mmcblk0p3-8.
[3585.266] I/O error, dev mmcblk0, sector 3751936 op 0x1:(WRITE)
[3585.267] Buffer I/O error on dev mmcblk0p3, logical block 327680, lost sync page write
[3585.276] JBD2: I/O error when updating journal superblock for mmcblk0p3-8.
[3590.323] EXT4-fs error (device mmcblk0p3): ext4_journal_check_start:87: comm balance_bot: Detected aborted journal
[3590.336] I/O error, dev mmcblk0, sector 1130496 op 0x1:(WRITE)
[3590.338] Buffer I/O error on dev mmcblk0p3, logical block 0, lost sync page write
[3590.340] EXT4-fs (mmcblk0p3): I/O error while writing superblock
[3590.341] EXT4-fs (mmcblk0p3): Remounting filesystem read-only
```

`-110` is `ETIMEDOUT`. `errcode 2` in `/sys/fs/ext4/` is `EXT4_ERR_EIO`.

What matters about the shape of this:

- **No degradation trail.** No CRC errors, no retries, no `Problem switching
  card into high-speed mode`, no timeouts of any kind before this. An hour of
  perfectly clean operation, then the card stops answering a *hardware reset*.
  Media wear does not look like this. It looks like a rising error rate.
- **The very first symptom is a failed HW reset**, not a failed data transfer.
  The controller had already decided something was wrong and tried to recover
  the card; the recovery is what timed out. Read the print order carefully: the
  block layer announces the reset attempt while handling a request that already
  failed, so the true sequence is *silent write failure → recovery → reset
  times out → I/O error surfaces*. The operation that actually failed first was
  a **DMA data write** to sector 3874952.
- **It never came back.** Not once. `mmcblk0: recovery failed!` appears **957
  times** between t=3585 and t=5266, and the card is never re-enumerated. A
  second `tried to HW reset card, got error -110` lands at **t=4134**, nine
  minutes after the first, and from there reads fail too (sector 1609728, and
  `blk_print_req_error` suppressing 638 callbacks in one burst). After that the
  box is a RAM disk with a shell on it: `/usr/bin/dmesg`, `/usr/bin/findmnt`,
  `/usr/bin/tail` and `/usr/bin/nc` all return `Input/output error`, and
  `journalctl` segfaults. That is why the dump had to be taken with
  `sudo cat /dev/kmsg` — nothing else was still readable.
- **The host cannot power-cycle the slot.** `sdhci-omap 48060000.mmc: supply
  pbias not found, using dummy regulator` and the same for `vqmmc`. So
  `mmc_hw_reset()` has no rail to toggle; all it can do is re-run the CMD0
  init sequence. A card that has latched up is therefore unrecoverable in
  software **by construction on this board**, and "957 failed recoveries" says
  nothing about whether the card or the host is at fault. Do not read it as
  evidence either way.
- **`mmc0: Got CD GPIO`** — there is a card-detect line, and it never fired.
  The card was never seen as removed or reinserted, which rules out the
  simplest mechanical story (the slot losing contact) for *this* event.

### 1.2 Who started balance_bot — nobody did, systemd did

Two lines name it:

```
[1611.674] omap8250 481aa000.serial: balance_bot sets custom speed on ttyS5. This is deprecated.
[3590.323] EXT4-fs error (...): comm balance_bot: Detected aborted journal
```

Ryan's account was that he never started it and it never came up. Both are
true, and the unit files explain how:

```
balance_bot_server.service:   After=balance_bot.service
                              Requires=balance_bot.service
```

**Starting the bridge pulls in `balance_bot`.** So the process existed without
anyone asking for it. It also came up inert — the unit's fallback is
`Environment="BOT_ARGS=-q -i none"`, deliberately no input source — which is
consistent with "it never successfully started" meaning *it never balanced*.

What the log does and does not establish about it:

- **It started exactly once.** Only one `sets custom speed on ttyS5` line in
  the whole boot. With `Restart=on-failure` set, a crash loop would have
  printed one per attempt. So: one start at t=1611.7, alive from then on.
- **It was alive at t=3590**, 33 minutes later — a dead process cannot be the
  `comm` on an ext4 error.
- **It does not follow that it was writing much.**
  `ext4_journal_check_start` fires on *any* attempt to start a transaction
  after the journal has already aborted, so this only identifies the next
  process to touch the filesystem, not the one that caused the failure. The
  write that actually failed, at t=3585 to sector 3874952, was writeback and
  could have originated anywhere.

Its only persistent writes are `/home/debian/balance_bot_imu.conf`,
`robot.conf` (written tmp-then-rename on tune changes) and
`/tmp/balance_bot.log`. An idle instance writes essentially nothing.

**Consequence for the power theory:** with no input source and nothing
balancing, the motors were almost certainly not drawing current during this
event. A motor-current brownout does not explain *this* failure. That does not
generalise to the other events, where the bot was running.

### 1.3 The strongest lead: shared EDMA between the UART and the MMC

**Three seconds before `balance_bot` took the RoboClaw port**, a `python3`
process closed a serial port and the kernel warned:

```
[1608.648] ------------[ cut here ]------------
[1608.648] WARNING: CPU: 0 PID: 806 at drivers/tty/serial/8250/8250_omap.c:1021 omap_8250_rx_dma_flush+0xb9/0xc0
[1608.655] CPU: 0 UID: 0 PID: 806 Comm: python3 Tainted: G           O        6.18.34-bone37
[1608.655] Call trace:
[1608.655]  omap_8250_rx_dma_flush from omap_8250_shutdown+0x27/0x124
[1608.655]  omap_8250_shutdown from uart_port_shutdown+0x27/0x34
[1608.655]  uart_port_shutdown from uart_tty_port_shutdown+0x4b/0x104
[1608.655]  uart_tty_port_shutdown from tty_port_shutdown+0x5b/0x74
[1608.655]  tty_port_shutdown from tty_port_close+0x1d/0x64
[1608.655]  tty_port_close from tty_release+0x6b/0x300
[1608.655]  tty_release from __fput+0x89/0x208
[1608.655]  __fput from sys_close+0x25/0x50
```

**That is `roboclaw_reset.py`, and it is part of normal startup:**

```
balance_bot.service:
  ExecStartPre=/usr/bin/python3 /home/debian/balance_bot/roboclaw_reset.py
  ExecStart=/usr/local/bin/balance_bot $BOT_ARGS
```

ExecStartPre closes the port, ExecStart opens it 3.0 s later. That matches the
two timestamps exactly.

This matters for reproducibility: the WARN did not come from some rare manual
action. **Every single `balance_bot` start runs this python script, which opens
`/dev/ttyS5` at a custom baud, talks to the RoboClaw and closes it.** If the
teardown is unsound, it is being exercised on every service start.

The WARN in `omap_8250_rx_dma_flush()` fires when the driver cannot pause the
RX DMA channel on shutdown — **the DMA engine refused the operation.** (Confirm
the exact condition against the 6.18 source before relying on the detail; the
function's job there is `dmaengine_pause()` on the RX channel and the WARN is
on its failure.)

Now the part that makes this interesting:

```
[3.791] edma 49000000.dma: TI EDMA DMA engine driver
[4.221] mmc1: SDHCI controller on 481d8000.mmc using External DMA
[4.227] mmc0: SDHCI controller on 48060000.mmc using External DMA
[4.297] mmc2: SDHCI controller on 47810000.mmc using External DMA
```

**All three MMC controllers and the OMAP UARTs share one EDMA controller** at
`0x49000000`. "External DMA" for SDHCI means exactly that — the SD/eMMC
transfers are run by the same TPCC/TPTC hardware that serves ttyS5.

So the hypothesis is: **the UART's RX DMA teardown leaves the EDMA engine in a
bad state, and MMC transfers on the same engine eventually time out.**

**Why this deserves priority over everything tried before — it is the first
theory that survives all four constraints simultaneously:**

| Constraint from Ryan's months of data | Does the EDMA theory fit? |
|---|---|
| Swapping SD ↔ eMMC never fixed it | Yes — `mmc0` *and* `mmc1` are both on the same EDMA |
| Kernel 4.1x went RO too, so did 6.x | Yes — the omap8250 DMA path is long-standing |
| No media degradation trail | Yes — the card is fine; the transfer never reaches it |
| Happens during robot operation | Yes — ttyS5 to the RoboClaw is only busy then |

**Counter-evidence, stated honestly.** The WARN is at t=1608 and the MMC
failure is at t=3585 — a 33-minute gap, not immediate causation. A leaked or
mis-parked EDMA channel could plausibly sit harmless until the wrong slot is
reused, but that is an assumption, not something the log shows. Also,
`WARN_ON_ONCE` prints **once per boot**, so the log cannot tell us how often
this actually happens; do not read "one occurrence" as "one occurrence".

**And the baseline has already narrowed it further.** `tools/edma_probe.sh`
recorded the healthy channel map: mmc0 sits on `dma0chan22/23`, **direct
mapped**, while the AM335x UARTs come off the `44e10f90.dma-router` crossbar.
A leaked UART channel can therefore never be re-handed to mmc0, which kills
the simple "leaked channel gets reused" mechanism outright. What survives is
only controller-level state shared across all channels — the PaRAM slot pool,
the TPTC queues, the error path. That is a real mechanism but a much weaker
one, and it should be held with a correspondingly weaker prior than the table
above implies on its own.

One thing the new capture does add in the theory's favour: **balance_bot
opened ttyS5 at t=1611 and the card died at t=3585.** Nothing failed during
the first 27 minutes of uptime, when no UART DMA was running. The apt/dpkg
event two days earlier (`unable to fsync updated status of 'libc-bin':
Input/output error`, visible at the top of `ro_dump.txt`) also happened with
balance_bot up. Two for two — which is suggestive and nothing more, since
balance_bot is up nearly all the time anyway. The clean way to spend that
observation is the 2×2 in §1.4 step 3.

### 1.4 How to test it — in order of cost

0. **Arm the beacon first — `tools/ro_beacon.sh --install`.** Everything below
   competes for one event per few days; the beacon costs nothing and improves
   *every* future capture, including events nobody is watching. One `<4>` line
   every 10 s to `/dev/kmsg` carrying battery volts, in-use EDMA channels,
   mmc0 presence, per-interval mmc/i2c interrupt counts and root rw/ro, plus a
   full EDMA + mmc0 snapshot dumped at the instant of the transition. It reads
   only `/proc`, `/sys` and `/run`, so the dying block device cannot block it.
   §1.6 is the proof that a sampler on this cadence survives the fault intact.
   **`v=` is the field that matters most**: the power theory is years old and
   has never once had a voltage trace at the moment of failure.

   Ryan — one question worth answering before anything else, because it is
   free and it splits the field cleanly: **when the box is in this state, does
   a warm `reboot` bring the card back, or does it take a power cycle?** Warm
   reboot recovering it means host-side state that a re-init clears (controller,
   DMA, clock) and the card is innocent. Only-power-cycle means the card itself
   is latched, and given §1.1 the host has no way to un-latch it.

1. **Cheapest, do this first.** Read the DMA engine's own state, on a healthy
   board and again right after an event if you can get a shell:
   ```
   sudo cat /sys/kernel/debug/dmaengine/summary
   ```
   Leaked or in-use channels with no owner would be direct support. Costs
   nothing and needs no reboot.

2. **Make it reproducible.** Open and close the RoboClaw port in a loop and see
   whether MMC errors follow. If the WARN can be provoked on demand, the whole
   fault becomes debuggable instead of a once-a-week ambush:
   ```
   for i in $(seq 200); do
       python3 -c "import serial; s=serial.Serial('/dev/ttyS5',38400); s.close()"
   done
   dmesg | grep -iE 'edma|8250|mmc'
   ```

2b. **The 2×2 that separates "the UART did it" from "it happens anyway".**
   Four long soaks, beacon armed, everything else identical:

   | | disk idle | disk hammered (`dd … conv=fsync` in a loop) |
   |---|---|---|
   | **balance_bot stopped** (ttyS5 never opened) | A | B |
   | **balance_bot running** | C | D |

   If the fault only ever appears in C/D, the UART DMA path is implicated and
   step 3 is worth its cost. If it appears in B, the UART is exonerated and
   this whole lead is spent — write that down and stop paying for it. Note
   that `balance_bot_server.service` has `Requires=balance_bot.service`, so
   "stopped" means stopping **both**.

3. **The decisive test: take the UART off DMA.** If ttyS5 runs in PIO and the
   read-only fault stops, that is the answer. On this SoC the UART only uses
   DMA when the device tree gives it `dmas`/`dma-names`, so a small overlay
   that deletes those properties from the `uart5` node forces PIO. The cost is
   more interrupts on a serial port that runs at RoboClaw baud rates — likely
   affordable, and measurable with `tools/cpu_top.py --threads` before and
   after. **This is the single highest-value experiment now open.**

4. If it survives all that, the fallback is the same trick in reverse: take
   *MMC* off external DMA and let it run PIO. Slower, but it isolates which
   side of the shared engine is at fault.

### 1.5 What is already ruled out for the RO fault

| Hypothesis | Killed by |
|---|---|
| Failing SD card / eMMC | Both media run; swapping never fixed it. **Do not revisit.** |
| Kernel 4.1x | The kernel was upgraded to 6.x *because* 4.1x also went RO |
| `unattended-upgrades` | apt ran Aug 2, Aug 8, Aug 16 — does not line up with events |
| journald write volume | `Storage=volatile` is set; the fault continued |
| Config saves / display logs | Both eliminated as write sources; fault continued |
| Media wear | No CRC or retry trail before the failure; abrupt from clean |
| The board froze / stalled / browned out | §1.6 — a userspace 30 s timer kept 256 ms accuracy straight through both storage deaths |
| Some other subsystem went down with it | Between t=61 and t=3585 the kernel logged **nothing at all** except the OLED spam, and nothing but mmc0 ever complained — not i2c, USB, wifi, Bluetooth, CAN or the IMU |
| The slot lost contact | `Got CD GPIO` is present and never fired |

Ryan's long-standing position has been that this is **power**. The EDMA theory
does not contradict a power cause so much as offer a mechanism that does not
require one; the two are distinguishable by test 3 above.

Worth being precise about what §1.6 does and does not do to the power theory:
it rules out a **board-wide** brownout, because the CPU, the timers and the
scheduler were untouched and no other 3.3 V peripheral so much as blinked. It
does not rule out a dip local to the card's own supply, which is exactly what
the beacon's `v=` field exists to catch.

### 1.6 The 30 s metronome — what the OLED bug accidentally proved

The 189 quirk rejections are not noise. They are a **userspace process on a
fixed timer, timestamped by the kernel**, running from t=52 to t=5507 — before,
during and 32 minutes after the failure. Measured:

- The interval sequence is one clean ramp — 2.2, 4.4, 6.1 … 28.1 s — then flat.
  That is exactly `Display.backoff()` (`min(30, 2 × fail_streak)`) counting up
  from a single cold start. **The service never restarted**, which
  `systemctl status` confirms independently: same invocation ID, PID 533, "1h
  21min" at 11:17.
- 174 steady-state intervals: **min 30.080 s, max 30.336 s — a 256 ms spread
  across 91 minutes.**
- The interval straddling the first storage death is **30.203 s**. The one
  straddling the second is **30.179 s**. Both sit inside the ordinary spread.

So at the moment mmc0 stopped answering, a Python process that wakes on a
timer, forks `systemctl`, and does an ioctl on `/dev/i2c-1` was keeping time to
a quarter of a second. There was no stall, no starvation, no IRQ storm, no CPU
reset, no clock glitch. **Whatever killed the card did not touch the rest of
the machine**, and any theory that requires the board to hiccup is finished.

The second lesson is the useful one: a 30 s sampler reading only virtual
filesystems sails straight through this fault. We have had proof of that
capability sitting in the logs the whole time, produced by a bug. `ro_beacon.sh`
is that capability on purpose.

### 1.7 What this capture does NOT establish

- **It is one event.** "The RO fault" has been treated as a single recurring
  thing for months. This is the first one with primary evidence, and there is
  no basis yet for assuming the eMMC-era events had this signature. If the next
  capture looks different, that is a finding, not a contradiction.
- **It cannot say whether the card or the host failed.** See the dummy-regulator
  point in §1.1 — the recovery path is crippled on this board either way.
- **It cannot say how often the 8250 WARN fires.** `WARN_ON_ONCE` is spent
  after the first print, every boot.
- **mmc1 and mmc2 were idle**, so the log offers no cross-check on whether the
  rest of the shared EDMA still worked after mmc0 died. A tiny periodic read
  against the eMMC would have answered the EDMA question outright. Cheap to
  add if the beacon does not settle it. **(Now in `ro_beacon.sh` as `emmc=`.)**

### 1.8 Second event, same day — and the first fault-time DMA snapshot

A second event landed the same evening, boot at ~14:41, and it repeats the
signature exactly: nothing from mmc0 for **3 h 47 min**, then a failed WRITE
(sector 3764872), `HW reset … error -110`, journal aborted, read-only 5 s
later, a second `-110` 115 s after that, `comm balance_bot` on the ext4 error
again. The metronome intervals straddling both deaths were **30.084 s** and
**30.091 s** — no stall, independently confirmed on a second event.

Four things this adds:

1. **The EDMA channel table at fault time is byte-identical to the healthy
   baseline.** Ten channels, same clients, 62 total. Nothing stranded, nothing
   leaked, no orphan UART channel. Taken ~5 minutes after mmc0 died. Combined
   with the crossbar argument in §1.3, **the "leaked EDMA channel" mechanism is
   now dead on evidence, not just on reasoning.**
2. **`49000000.dma_ccerrint` reads 43.** The EDMA channel-controller *error*
   interrupt is not zero. There is no baseline for it yet, so 43 could all be
   from boot — but it is the first hard number pointing at the engine, and
   whether it *steps at the moment of failure* is now a one-line test. The
   beacon logs `ccerr=` and `dccerr=` for exactly this.
3. **The WARN→death gap does not replicate.** 1977 s in event A, **11051 s**
   in event B — 5.6×. Whatever the relationship between the UART teardown and
   the storage failure, it is not a fixed incubation, and a simple accumulate-
   until-it-breaks story does not fit.
4. **`4802a000.i2c` has taken 0 interrupts this boot.** Not one transaction has
   reached the i2c1 hardware since power-on, despite the OLED probing every
   30 s — the third independent confirmation that those rejections never touch
   the wire (§3.1).

Battery at capture: **12.445 V, status ok.** Not a pack-level brownout.

**The `emergency_ro` trap.** During this event `/proc/mounts` still read:

```
/dev/mmcblk0p3 / ext4 rw,noatime,errors=remount-ro,emergency_ro 0 0
```

Still `rw`. Modern ext4 sets an emergency-read-only superblock flag instead of
rewriting the mount flags, so a naive rw/ro check **misses the event entirely** —
`ro_beacon.sh`'s original trigger would have sailed straight past the thing it
exists to catch. Fixed, and `errors_count` is now a second independent trigger.
Worth remembering the general shape: this is the third time on this project a
detector has quietly reported success while doing nothing (netconsole, the OLED
probe, now this).

---

## 2. netconsole — was never actually capturing (now fixed)

`tools/netconsole_setup.sh --persist` wrote `netconsole` into
`/etc/modules-load.d/`. That is loaded by `systemd-modules-load.service` in
early boot, at **t≈20 s**. The `rtw_8821au` dongle's `wlan0` does not exist
until **t≈57 s**. Every boot:

```
[20.039] netconsole: local port 6665
[20.041] netconsole: interface name 'wlan0'
[20.066] netpoll: netconsole: wlan0 doesn't exist, aborting
[20.067] netconsole: Not enabling netconsole for cmdline0. Netpoll setup failed
[20.079] netconsole: network logging started          <-- not true
```

The module initialised fine with **zero live targets** and printed a success
line anyway. The script printed `netconsole loaded.` It looked armed. It
captured nothing, and at least one RO event went unrecorded because of it.

**Rewritten.** `--persist` now installs a `netconsole-bot.service` ordered after
the network, which re-runs the script at boot so the addresses and the
receiver's MAC are re-resolved rather than baked in, and which polls for the
link itself (`--wait 180`) rather than trusting `network-online.target` — with
`iwd` there may be no wait-online implementation enabled at all. Arming now
goes through **configfs** in preference to module parameters, because writing
`enabled` returns a real errno while the module-parameter path cannot report
failure. Two new modes:

```
sudo ./tools/netconsole_setup.sh --status      # is a target actually ENABLED?
sudo ./tools/netconsole_setup.sh --unpersist   # removes the unit AND the old
                                               # modules-load.d/modprobe.d files
```

**Run `--unpersist` once before `--persist`** on this board, to clear the old
broken files — otherwise the early load races the new unit.

Never trust "loaded". Verify end to end, every time:

```
echo "netconsole test from $(hostname)" | sudo tee /dev/kmsg
```

---

## 3. OLED — my probe could never have succeeded

The kernel log carries this **189 times in one boot**, at exactly the 30 s
back-off cadence of our own retry loop:

```
i2c i2c-1: adapter quirk: no zero length (addr 0x003c, size 0, write)
```

Every one of them is `bus.write_quick(self.addr)` in `BbbOled._probe()`.
`write_quick()` is a **zero-length** transfer, and the AM335x OMAP adapter
declares `I2C_AQ_NO_ZERO_LEN`, so the i2c core rejects it before it reaches the
wire. **The probe failed 100% of the time whether or not a panel was
attached.** That is my bug, introduced with the fd-leak fix.

Fix: `bus.write_byte(self.addr, 0x00)` — `0x00` is an SSD1306 command-mode
control byte with no command after it, so a real panel accepts it and does
nothing, while an absent one still raises `OSError`.

**Applied 2026-08-23** in `oled-utils/bbb_oled.py` (`_probe()`, ~line 205),
with the docstring corrected — it previously argued *for* `write_quick()` on
the grounds that it is what `i2cdetect` uses, which is true and irrelevant
here. Compiles clean. **Not deployed to the board yet** — needs a sync and
`systemctl reset-failed bbb_oled && systemctl restart bbb_oled`. Confirm with:

```
journalctl -u bbb_oled -n 20 --no-pager
sudo dmesg | grep -c 'no zero length'      # should stop growing
```

**Expect the panel to stay dark.** Last full bus sweep found `i2c-1`
completely empty; `i2c-2` has only the IMU stack (`0c` magnetometer, `68`
MPU-9250, `UU 76` barometer) and `i2c-0` has PMIC/EEPROM (`UU 24`, `UU 50`).
The panel answers on no bus, which is a wiring or power problem. What the fix
buys is an **honest** probe and a kernel log that is not being spammed every
30 s. Also note `i2c-1` runs at 100 kHz while `i2c-0`/`i2c-2` run at 400 kHz —
a 1 KB SSD1306 frame is ~100 ms at 100 kHz vs ~25 ms at 400. Relevant only
once the panel exists.

### 3.1 What the RO capture says about i2c-1 — the bus is fine

Three independent pieces, and they agree:

1. **The controller is up and correctly muxed.** `omap_i2c 4802a000.i2c: bus 1
   rev0.11 at 100 kHz`, and the pinmux confirms ownership:

   ```
   pin 86 (PIN86): 4802a000.i2c (GPIO UNCLAIMED) function pinmux_bb_i2c1_pins
   pin 87 (PIN87): 4802a000.i2c (GPIO UNCLAIMED) function pinmux_bb_i2c1_pins
   ```

   Pins 86/87 are offsets 0x958/0x95C — `spi0_d1`/`spi0_cs0` in their I2C1
   mode, the standard mux. "GPIO UNCLAIMED" only means no GPIO consumer also
   wants them; it is not a warning. **The pinmux theory is dead** — the pins
   belong to the I2C controller.
2. **The wire is being driven and nothing answers.** `sudo i2cdetect -y -r 1`
   sweeps clean, all `--`. `-r` is SMBus read-byte, a real one-byte transfer,
   so unlike the probe it actually reaches the bus. And across the whole
   91-minute capture there is **not one i2c error, timeout or arbitration-lost
   message** — only the quirk rejections. A bus with SDA or SCL held low does
   not fail silently on this driver; it logs timeouts. So the bus is idling
   high and transactions are completing with a NAK.
3. **The 189 rejections say nothing at all about the hardware.**
   `i2c_check_quirks()` rejects a zero-length write inside the i2c *core*,
   before the OMAP driver clocks a single bit. Every one of those lines is a
   software refusal. They were never evidence about the panel, in either
   direction.

Put together with "I saw it work once": the bus is healthy, the address is
right, and the panel is intermittently — now persistently — not answering.
That is the panel's power or its connector, not the BeagleBone. The next
measurement is the errno, which separates the two remaining cases in one
command:

```
sudo i2cget -y 1 0x3c
```

- `Remote I/O error` (EREMOTEIO, 121) → bus toggling, nobody home. Panel power
  or a broken conductor. Reseat, wiggle, meter the panel's 3V3 and GND at the
  module end.
- `Connection timed out` (110) or arbitration lost → a line is being held down.
  Pull-ups, a half-powered panel back-driving SDA, or a short.

`oled_check.py` currently answers "did anything ACK", which is a strictly
weaker question. Teaching it to report the errno is a small change and it is
the difference between "nothing there" and "here is which half of the problem
you have".

### 3.2 The service says `active (running)` and always will

`ConditionPathExists=/dev/i2c-1` is satisfied by the adapter existing, which
has nothing to do with the panel. `ensure()` then catches every probe failure,
logs it once, and sleeps. The process never exits, so `Restart=always` and
`StartLimitBurst` never engage and `systemctl status` shows a healthy green dot
forever — 5.168 s of CPU, 14.1 MB, "active (running) since …", panel dark since
installation.

That is the netconsole mistake in a different costume: a component reporting
success for a job it has never once done. Worth fixing the same way — after N
minutes with zero successful connects, either exit non-zero so it shows up in
`systemctl --failed`, or publish state to `/run/oled_status.json` where the
dashboard can see it. The bug is not that it retries; retrying is right. The
bug is that it looks fine while doing it.

---

## 4. Position hold — where the tuning stands

Full detail and the running table are in **`TUNING_LOG.md`**. Summary:

- Best measured setting is **`scale_d = 40`, `vel_scale_stop = 5`** (320 s run,
  `|encErr|` 22.6, pitch sd 1.69). `35 / 5.5` centres better (18.6) but is
  slightly less calm.
- Stability is **bounded on both sides** in `vel_scale_stop`: 3.5 rings, 25
  falls over, 5–5.5 works. The surviving explanation for the upper bound is the
  RoboClaw's measured 50–100 ms of velocity lag.
- **All signals lock to one period** (1.95 s at 40/5) — it is a limit cycle,
  not noise.
- **The pitch integrator is railed** at its −0.02 clamp for 9.6% of samples and
  negative 95.7% of the time. That is a trim error, not a gain problem.
  **Re-zero the IMU** before spending more time on gains — this is the next
  thing to do on tuning and it is cheap.
- **Zones are inert.** `zone_c = 500` against errors that never exceed ~95
  ticks means `scale_d` is the only scale ever selected; `scale_a/b/c` are dead
  config. The ordering (A=60, B=80, C=200, D=50) is not monotonic, so Ryan's
  intent per zone needs establishing before the thresholds are touched.
- **`drive_mode` disagrees**: bot reports 1, UI reports 0, on the last three
  runs. The dashboard has been wrong about drive mode the whole time.

### The velocity estimator was tried and rejected — do not retry it blind

A least-squares slope estimator over `enc_pos` was built to replace the
RoboClaw's velocity reading. It made the bot unbalanceable. Measured head to
head, the RoboClaw reading is **1.59× smoother** than a 6-tick slope; no window
length wins (`TUNING_LOG.md` has the table). The scaffolding is still in
`src/robot.c` behind `POS_VEL_USE_LSQ` (0 = RoboClaw controls, the only setting
that balances) with all three candidates logged to telemetry. **Strip it when
it stops being useful.**

The lesson recorded there is worth repeating: both wrong predictions came from
a simulation whose only noise source was encoder quantisation, and the real
error was that *the incumbent was never characterised before being replaced*.

### Untested

- `35 / 5` — separates the last change into its two parts
- `30 / 5` — does centering keep improving, and at what cost
- anywhere below `vel_scale_stop = 4` or above `6`

---

## 5. Resolved this month, with evidence (do not re-litigate)

### Telemetry collapse when the RC transmitter was powered — SOLVED

**Cause: 2.4 GHz channel congestion.** The AP was on channel 5 (2432 MHz),
overlapping both channel 1 and channel 6. The bot's *transmit* path collapsed
to MCS 1–2 while its *receive* path ran MCS 6, at a perfectly healthy −64 dBm.

**Fix: moved the bot to 5 GHz** (`TakeFlight_501`, ch 36 / 5180 MHz) with
`iwctl`. Both directions now run 117 Mbps VHT-MCS 3. The dashboard holds 40 Hz
with an empty send queue and the transmitter has no effect at all.

**The RC transmitter was never the cause.** It cost one MCS step on a link that
was already broken with it switched off. That misattribution burned a week.

What actually isolated it: **per-client** rate and queue-depth logging in
`server.js`. A loopback client held 40 Hz at `buf=0` while a Mac and an iPhone
stalled *identically* at ~262 KB on the same bridge at the same instant. Two
different devices failing at the same byte count is not a software property.

Process lessons worth keeping:

- Channel 5 was noticed early, written down as "worth moving regardless", and
  then not pursued. The note was right; the follow-through was not.
- A clean `ping` was over-read as exonerating the link. One 64-byte packet per
  second says nothing about a path that fails at 30 kB/s.
- Several diagnostics were built on metrics `rtw88` never populates: `tx
  retries` is a permanent 0 and the noise floor is a `-256` sentinel, which
  produced a bogus 198 dB SNR in the HUD. **Verify a metric moves before
  building a display on it.** The only responsive metric on this driver is `tx
  bitrate` / MCS.
- The decisive instrument compared *two clients on the same bridge* rather than
  examining one client harder.

### IMU I2C polling storm — SOLVED

`mpu_thread()` polled `INT_STATUS` every 2 ms and read the FIFO only when it
announced data: 500 register reads/s to collect 100 packets, each an
`I2C_RDWR` ioctl carrying two i2c messages. Replaced with a phase-locked
absolute `clock_nanosleep`, re-locked to each successful read, reading
`FIFO_COUNT` directly; `INT_STATUS` is consulted only after 8 consecutive empty
reads (a full FIFO reports count 0 on this part, so an empty read is
ambiguous).

| | before | after | |
|---|---|---|---|
| `irq/32-4819c000.i2c` (I2C2 = IMU bus) | 1972/s, 7.2% | 646/s, 2.8% | −67% |
| `balance_bot` DMP thread | 2110/s, 9.7% | 603/s, 3.3% | −71% |
| board total | 6137/s | 3133/s | −49% |

A stubbed-i2c harness predicted 254 ioctls/s; back-calculating from the board's
646 IRQ wakes gives ~248 — **accurate to 2%**, so further estimates on this
path can be trusted.

### `LOG_WARN_EVERY` has never once throttled anything — OPEN

The 2026-08-24 partial archive recovered `/tmp/balance_bot.log`:

```
577,657,108 bytes    3,452,153 lines    154 boot markers
```

Overwhelmingly one message, repeating every 16-20 ms:

```
[WARN][659610] motor_hal_roboclaw: encoder read failed (-2)
[WARN][659630] motor_hal_roboclaw: encoder read failed (-2)
```

That call site is `src/motor_hal_roboclaw.c:114`, and it is **already throttled**
in the source: `LOG_WARN_EVERY(2000, ...)`. The macro (`include/debug_config.h:457`)
looks correct — it fires at most once per 2000 ms and appends
`[+N more suppressed]` when it has skipped any.

**Scanning all 577 MB: `more suppressed` appears ZERO times.** Not once, across
154 boots. So the throttle has never taken its else-branch — every single
occurrence was logged, and `_lwe_skipped` never incremented.

Two candidates, and one command separates them:

```
strings /usr/local/bin/balance_bot | grep -c 'more suppressed'
```

- **0** → the installed binary predates the macro. `make` was run without
  `make install`, which is the environment gotcha already documented at the top
  of this file. Rebuild, install, done.
- **≥1** → the macro is in the binary and still not throttling, which points at
  the sentinel: `_lwe_last == 0` is both "never fired yet" and a legitimate
  timestamp value. If `rc_nanos_since_boot()` returns 0 — on error, or on the
  first call inside the first millisecond — `_lwe_last` stays 0 forever and the
  `_lwe_last == 0` branch fires every time, which produces exactly this
  signature: everything logged, nothing counted as suppressed. Fix by using a
  separate `static bool _lwe_primed` rather than overloading 0.

`[WARN] Unknown command type` is the second-largest source (4305 occurrences in
the first 4 MiB) and is not throttled at all.

**This also corrects §1.2.** "An idle instance writes essentially nothing" was
inference and it was wrong: balance_bot appends to `/tmp/balance_bot.log` at
tens of lines per second, `/tmp` is **not** a tmpfs on this image (no `/tmp`
line in `/proc/mounts`), and nothing rotates or clears it. That is a continuous
write load on the card, half a gigabyte of it. It is not the cause of the
read-only fault — §0.5 settles that — but it is gratuitous, it would have
blown the eMMC fit, and it is why `clone_to_emmc.sh` excludes `/tmp`.

Worth doing regardless of which candidate above is right: make `/tmp` a tmpfs
(`systemctl enable tmp.mount`, with a size cap — 512 MB of RAM total, so
`size=64M`). Fix the throttle first, though: a 551 MB log in a tmpfs is an OOM,
not a fix.

### Smaller fixes

- **`lastConfig` scope error** (`server/server.js`) — declared inside
  `connectToBalanceBot()` but read from the websocket `connection` handler, so
  **every browser connect threw `ReferenceError`** and the dashboard showed
  "not connected" while every service ran normally. Now at module scope, with a
  comment explaining why. `node --check` cannot catch this; a harness that
  actually connects a client can, and now exists.
- **Config never reaching browsers** — the bot sends `config` once, to the
  bridge, at IPC connect. Any browser connecting later got nothing and showed
  built-in defaults while claiming to show the robot. The bridge now caches and
  replays it per connection.
- **Websocket backpressure** — `readyState === OPEN` is not a liveness test; a
  sleeping phone stays OPEN for minutes. Added a 256 KB high-water drop, 15 s
  stall eviction, and a ping/pong heartbeat. Telemetry is latest-state, not a
  log: a slow client should miss frames, never delay everyone else.
- **Graph buffer** (`web/bbot_dashboard_v6.html`) — `MAX_PTS = 600` was 60 s at
  the old 10 Hz rate and silently became a 30 s cap at 20 Hz. Now derived from
  `windowSec × 100`. Draw path gained min/max-per-pixel-column decimation
  (plain striding drops single-sample spikes, which on a PID trace are the
  point). Removed `Math.min(...allVals)` — spreading a 60 s buffer can exceed
  the engine's argument limit and throw `RangeError`.
- **OLED fd leak** — luma's `i2c()` opens `/dev/i2c-N` then raises on a failed
  address probe, leaving the descriptor held only by the traceback. Reached
  EMFILE in ~2 h with 164 MB RSS on a 512 MB board. Fixed by probing on a
  descriptor we own and close in `finally`; 0 leaks over 2000 failed connects.
  (The probe *itself* was still broken — see §3.)
- **`cpu_top.py`** silently dropped any process whose PID changed mid-sample,
  so measuring right after a deploy discarded `balance_bot` and `node` and then
  printed "1365/s is healthy". Now reports what restarted and refuses a verdict
  when the bot's processes are missing.
- **`oled_check.py` dependency check** was wrong in *both* directions before
  landing: v1 asked "can `debian` import luma?" and passed a service dying on
  `ImportError` (because `~/.local` is hidden by `ProtectHome=yes`); v2
  rejected any module under `/home` and blocked a *working* install. Now probes
  with `PYTHONNOUSERSITE=1`, which is what the sandbox actually leaves visible.

---

## 6. Open items

| # | Item | State |
|---|---|---|
| 0 | **Arm `tools/ro_beacon.sh`** | §1.4 step 0. Do this before any other RO work — it improves every future capture, including unattended ones, and carries the voltage trace the power theory has never had. |
| 1 | ~~**EDMA / UART-DMA hypothesis**~~ | **CLOSED — §0.5.** The failure reproduces in U-Boot at power-on, where none of that stack exists. Do not spend another hour here. The DT overlay experiment is cancelled. |
| 1a | **Read the SD card in a USB reader on the Mac** | §0.5. Exonerates the card or does not, costs nothing, and is the only thing standing between "the socket / its 3.3 V" and "the card" as the remaining explanation. |
| 1b | **Restore a bootable eMMC** | §0.5. `** No partition table - mmc 1 **` means a card that will not init is a brick, not a nuisance. There is no fallback today. |
| 1c | **Meter 3.3 V at the SD socket during power-on; inspect the socket** | §0.5. Voltage select failing is an electrical event in the card-init path. Contacts, solder joints, retention, supply. |
| 1b | **Answer: warm reboot or power cycle?** | §1.4 step 0. One sentence from Ryan splits host-side state from a latched card. Costs nothing. |
| 2 | **Deploy the OLED probe fix** | §3. Source is patched; sync + `reset-failed` + restart, then check the quirk-rejection count stops growing. Then `i2cget -y 1 0x3c` for the errno (§3.1). |
| 2b | **Move swap off the SD card** | Swap is `/dev/mmcblk0p2`, on the same device that dies. Once it is gone, any page fault against a swapped-out page becomes SIGBUS — which is why `journalctl` segfaulted and half of `/usr/bin` returned `Input/output error` during the capture. `zram` swap on a 512 MB board costs little and keeps the shell alive long enough to take evidence. |
| 2c | **Make bbb_oled report honestly** | §3.2. It says `active (running)` while having never connected. |
| 2d | **`LOG_WARN_EVERY` throttles nothing** | §5. 577 MB / 3.45 M lines / 154 boots, zero `more suppressed`. Start with `strings /usr/local/bin/balance_bot \| grep -c 'more suppressed'`. |
| 2e | **`/tmp` is on the SD and never cleared** | §5. Make it a tmpfs with `size=64M` — *after* 2d, or you trade a disk leak for an OOM. |
| 2f | **Why do encoder reads fail at all?** | §5. `motor_hal_roboclaw: encoder read failed (-2)` is the single most common line in the log by an order of magnitude. The throttle bug hid how often, but not that it happens. |
| 3 | **Re-verify netconsole persistence** | §2. `--unpersist`, then `--persist`, then reboot, then `--status` and the `/dev/kmsg` test. |
| 4 | **Re-zero the IMU** | §4. Pitch integrator is railed at its clamp 9.6% of the time. Cheap, and blocks further gain work. |
| 5 | **Host telemetry in `server.js`** | Ryan asked for information out of the bridge when `balance_bot` is not running. Proposed: a 1 Hz `{"type":"host"}` message (uptime, load, mem, wifi MCS, fs rw/ro, service states). **Not built — never answered whether to build it or list it.** |
| 6 | **UI backlog** | `web/UI_BACKLOG.md`, 7 items. Three open questions: which header to keep, confirm the PID/TUNE merge, columns vs stacked sections. Go slow. |
| 7 | **RC recovery mode** | At `eff_angle > 15°` the bot cuts motors and calls `motor_hal_standby(1)` and cannot recover, because the arms hold it at 15–18° — above the 10° that would restore `trying = 1`. E-stop is NOT asserted on a fall, so the RoboClaw stays live and this is contained: in the OOB branch, allow stick → direct duty clamped to ~±0.35, bypassing the balance PID and position controller. **Blocked on one decision: what arms it** — a spare RC channel (safest), a stick-held-past-70%-for-500 ms gesture, or automatic (not recommended: the wheels go live the instant it topples, which is when your hands are near it). |
| 8 | **Test `scale_d = 35` with `vel_scale_stop = 5`** | §4. Never actually run. |
| 9 | **Strip the LSQ scaffolding** | `src/robot.c`, when it stops being useful. |
| 10 | **Perf HUD shows `temp 0°C`** | The thermal-zone read (`sys_temp`) returns 0, so the temperature threshold is dead. Cosmetic but misleading. |
| 11 | **DMP loop, one more halving** | ~248 ioctls/s at ~2.5 per sample against a floor of 2. Combining the `FIFO_COUNT` read and the packet read into a single 4-message `I2C_RDWR` (`[write 0x72][read 2][write 0x74][read 32]`) gets to ~110/s; projected `irq/32` 646 → ~290/s, board total 3133 → ~2400/s. When count ≠ `packet_len`, discard and fall through to the existing drain path — do **not** simply drop the count read, or you read the oldest packet and silently accumulate FIFO backlog into the PID. |
| 12 | **`/run/rf_status.json` sampler** | A 1 Hz process writing `iw` output to a file `telemetry.c` reads — same pattern as `batt_monitor` → `/run/batt_status.json`. How to get tx bitrate/MCS into the dashboard without a subprocess in the control-loop process. |

---

---

## Claw power-up reboots the Blue — separate bug, NOT the RO fault

Stated by Ryan, 2026-08-24. **Ryan's call: this is a separate annoyance, not
the read-only fault** — and an earlier draft of this section made it the
leading RO theory on nothing but a plausibility argument. It is filed here as
its own bug. The supporting evidence for keeping them apart: the reboot is a
discrete power-up event, while RO arrives minutes into a run with the claw
already powered; and at RO time *nothing else on the board complains at all* —
no USB, wifi, i2c or IMU — which is not how a supply transient behaves.

- The RoboClaw is powered **directly from the battery**.
- The BeagleBone Blue is on **its own 5 A regulated supply**.
- **Powering the claw reboots the Blue.**

Two separate supplies, and one resets the other. That is not a subtle effect —
it is a hard fault that is already demonstrable on demand, and it means there
is a path between the two systems carrying enough energy to reset an ARM SoC.
**Grounding is NOT the answer — Ryan confirmed the bonding is already there**,
three ways over: through the power supply rails, through the serial harness
ground, and through a separate ground for the e-stop. An earlier draft of this
section guessed at a missing ground bond. That guess was wrong; delete it from
your head.

With the grounds solid, a reset on claw power-up points at the **source**, not
the return path:

1. **Shared battery, and inrush.** The Blue's 5 A supply is *regulated*, but it
   is regulated from the same battery the claw sits on. A RoboClaw's bulk
   capacitance is in the thousands of µF, and charging it draws tens of amps
   for a few milliseconds. Across the battery's internal resistance and the
   harness, that is a large momentary sag — and if it takes the regulator's
   input below dropout, even briefly, the Blue resets. This explains the
   symptom completely without requiring anything to be wrong with the grounds,
   and it is the leading candidate.
2. **The UART (ttyS1) and the e-stop line (GPIO1_25).** With one side powered
   and the other not, current flows into the unpowered side's logic pins
   through its ESD diodes — structures rated for microamps. Still worth series
   resistors, but this is now the second theory, not the first.
3. **Ground loops.** Three parallel ground conductors of differing impedance
   between two chassis form loops, and any di/dt nearby induces circulating
   current. Real, and worth knowing about for noise — but loops cause noise,
   not resets. Do not chase this one first.

### The binary test — swap where the Blue gets its power

Everything above reduces to one question: does the disturbance arrive through
**the battery**, or through **the signal wires**? One experiment separates
them and needs no scope.

**Run the Blue from a completely independent source** — a bench supply, or a
second battery with no connection to the drive pack — leaving the claw on the
robot battery exactly as it is. Then power the claw.

- **Blue no longer reboots** → it is conducted through the shared battery.
  Inrush, dropout, sag. Fix at the source (below).
- **Blue still reboots** → the battery is not the path, and it is arriving on
  the serial/e-stop wires despite the bonding. Series resistors and isolation
  become the priority.

### What to fix, once the test says which

If it is the battery path:

- **Inrush limiting on the claw** — NTC limiter, or a precharge resistor
  bypassed by a contactor once the caps are up.
- **Hold-up for the Blue's regulator** — a series diode plus a decent
  electrolytic on the regulator's INPUT, so a millisecond-scale sag on the
  pack never reaches it. Cheap, and it addresses the mechanism directly.
- **Separate the feeds** — the Blue on its own pack or a dedicated BEC, which
  is what the independent-supply test is already simulating.

If it is the signal path:

- **Series resistors on ttyS1 TX/RX** (100–330 Ω), and on the e-stop line, to
  limit ESD-diode current when one side is unpowered.
- **A digital isolator on the RoboClaw link** if there is room for it.
- **Check for back-powering with a meter**: Blue off, claw on, look for
  voltage on the Blue's 3.3 V rail. Anything there arrived through signal pins.

A scope on the Blue's regulator input during claw power-up shows the sag
directly and would settle it in one capture. A meter is too slow to see it.

---

## 7. Tools

| Tool | What it answers |
|---|---|
| `tools/sock_rate.py [secs]` | per-type rates straight off the unix socket — is the *bot* emitting cleanly? |
| `tools/ws_rate.py [secs] [url]` | the same one hop out, at the websocket. Run it **on the bot** against `ws://localhost:8675` with a browser also connected — that comparison is what solved the telemetry fault |
| `tools/cpu_top.py [secs] [--threads]` | which process/thread is switching or burning CPU |
| `tools/rf_check.py [secs]` | which link metrics actually respond (built because the shell was unusable in the failure state) |
| `tools/sbus_load.py` | interrupts/ctxt/serial errors |
| `tools/netconsole_setup.sh` | kernel log off-board over UDP; `--status` to check it is really armed |
| `tools/ro_beacon.sh` | 10 s heartbeat to kmsg — volts, EDMA channels, mmc0, irq deltas, rw/ro — plus a full snapshot at the moment the fs flips. `--install` for the unit |
| `tools/edma_probe.sh` | does opening/closing the RoboClaw port leak EDMA state? `--reset N` is the faithful reproduction |
| `tools/bot_up.sh` | bring services up one at a time, measuring each |
| `tools/bot_recover.sh` | recovery after a lockup |
| `tools/bbot_watch.py` | continuous recorder |
| `tools/analyze_tune.py` | CSV run analysis |
| `tools/trim_from_log.py` | derive a trim offset from a recorded run |
| `oled-utils/oled_check.py` | walks the OLED chain link by link, sweeps every I2C bus |

**Companion documents:** `TUNING_LOG.md` (the accumulating tuning map and the
rejected-approaches record) and `web/UI_BACKLOG.md` (dashboard work, captured
from Ryan's review, nothing implemented).
