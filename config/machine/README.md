# config/machine — this board's calibration

Snapshots of files that live outside the repo, are written by the firmware at
runtime, and are expensive or impossible to recreate from scratch.

These are **not defaults**. They describe one physical robot: this IMU, in this
chassis, on this network. Do not copy them to a second board and expect it to
balance.

| file | installs to | written by |
|---|---|---|
| `balance_bot_imu.conf` | `/etc/balance_bot_imu.conf` | firmware, on `zero_imu` |
| `balance_bot.default` | `/etc/default/balance_bot` | you, by hand |

## Why bother

`balance_bot_imu.conf` holds `pitch_offset` — the raw IMU angle at which the
robot actually balances. It is derived empirically: stand the bot up, press
Zero IMU, then trim out the residual by watching where the position hold
settles. Getting it wrong by one degree makes the bot creep and burn a fifth of
D2's correction authority standing still.

It exists in exactly one place on a running board and is erased by a reflash.

## Use

Run on the bot, after any calibration change:

    make save-config        # /etc -> here, then commit

On a fresh board, after `make install`:

    make install-config     # here -> /etc, only if absent

`install-config` will not overwrite an existing `/etc` file. A live calibration
always beats a committed one that may be months old — delete the target by hand
if you genuinely want the snapshot back.

## Not stored here

WiFi credentials (`/var/lib/iwd/*.psk`). Those are secrets and do not belong in
a git repo. Re-enter them with `iwctl` on a fresh board.
