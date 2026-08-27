# Dashboard UI backlog

Captured 2026-08-22 from Ryan's review of `bbot_dashboard_v6.html`.
Nothing here is implemented. Work it one item at a time.

---

## Principles he stated

- **No sliders.** They are the wrong control for a numeric parameter. Preferred
  model: **+/- buttons against a scale, AND a field you can type an exact value
  into.** Both, not either.
- **Panels, not pages.** Things should be condensable, closable, and
  combinable — rather than one big scrolling column per tab.
- **Not busy.** Density is fine; clutter is not. Analysis output in particular
  must not shout over the controls.
- **Go slow.** One change at a time, reviewed.

---

## 1. Header — remove the duplication

The top strip and the card row show **the same four values**:

```
strip:  BATT 11.74V   CLAW V 11.6V   ANGLE -17.9°   LOOP 99
cards:  BATTERY 11.74V | CLAW V 11.6V | ANGLE -17.9° | LOOP HZ 99
```

Keep one. Open question: which?

- The strip is always visible and compact — good for the things you glance at
  constantly (connection, angle, loop).
- The cards are large and readable across a room — good if the bot is on the
  bench and the laptop is at arm's length.

Probably: keep the strip, delete the cards, and reclaim the vertical space for
the arm/e-stop row and the mode selectors. **Needs a decision before building.**

---

## 2. Merge PID and TUNE into one tab

He wants the PID tab renamed to something like "Tune" — but **there is already
a TUNE tab** in the nav (CONTROL · PID · GRAPH · CLAW · RC · DEBUG · TUNE ·
SETTINGS). So this is a merge, not a rename, and that is consistent with the
next item: he wants the tune *analysis results* to appear in the same place as
the tune *parameters*.

Target shape for the merged tab:

- **Parameters at the top.** All controllers visible **at once** — pitch,
  position hold, yaw — not one at a time behind a selector.
- **Analysis results below**, quiet by default.

Open question: what does "all controllers at once" mean on a laptop screen —
three columns side by side, or three stacked collapsible sections? Columns are
denser; stacked sections survive a narrow window better.

---

## 3. Numeric input control to replace every slider

One component, used everywhere a parameter is set:

- `−` and `+` stepper buttons
- a visible scale/range indication so you know where the value sits in its
  useful span
- **a text field that accepts a typed value**
- sensible step sizes per parameter (`max_angle_rate` wants 0.01;
  `zone_a` wants 100)

This is the highest-leverage item — it touches Position Hold, PID gains, SBUS
config and rates, and it is what he actively dislikes about the current UI.

Note: the current Position Hold panel is the worst offender — 13 sliders, and
`Scale D` has already broken its own layout (the slider and value box wrapped
onto separate lines below the label).

---

## 4. Detachable tune panels alongside the live graphs

Be able to show a tune panel **next to the graphs** rather than having to switch
tabs to change a value and switch back to see the effect. That round trip is the
core friction in tuning.

Open question: floating/draggable panels, or a fixed split (graphs left, one
pinned tune panel right)? A fixed split is much less work and probably enough.

---

## 5. Keep the bottom HUD as-is — it is the model

He explicitly likes it:

- always-visible corner window
- **click to collapse into a summary**, and the summary surfaces only what is
  wrong

**Use this as the pattern for the panel work above** rather than inventing a
second interaction model. Whatever "closable panel" means elsewhere in the
dashboard should behave like this does.

---

## 6. Run timer on the record control

Elapsed time while recording, shown on/next to the record button.

Why it matters beyond convenience: runs are being compared against each other,
and length is part of whether a comparison is fair. The 50/5 baseline was 62 s
and the 40/5 run was 320 s — the longer one is far more trustworthy, but that
only became apparent after the fact, from the CSV. Ryan did not know he had
recorded five minutes until it was analysed.

Small scope: elapsed `mm:ss` from the moment recording starts. Worth also
showing sample count, since that is what actually determines how much the
statistics can be trusted.

---

## 7. Condense / split existing panels

General pass, informed by the above. Candidates:

- CONTROL tab: mode selectors (Robot Mode / Drive / RoboClaw Mode) are three
  full-width rows with long explanatory paragraphs. The explanations are good
  and worth keeping — but as tooltips or a collapsed "?" rather than permanent
  body text.
- Position Hold: 13 controls in one flat list with no grouping beyond the small
  caps headers. Zone thresholds, scale factors, velocity scales and limits are
  four distinct concepts that could each be a collapsible group.

---

## Also outstanding (not UI)

- **RC recovery mode.** At `eff_angle > 15°` the bot cuts motors and calls
  `motor_hal_standby(1)`, and cannot recover on its own because the arms hold it
  at 15–18° — above the 10° threshold that would restore `trying = 1`. The
  e-stop is NOT asserted on a fall (only by the E-STOP button), so the RoboClaw
  stays live and this is a contained change: in the OOB branch, allow stick →
  direct duty, clamped to ~±0.35, bypassing the balance PID and position
  controller entirely.
  **Blocked on one decision:** what arms it — a spare RC channel (safest), a
  stick-held-past-70%-for-500ms gesture, or automatic (not recommended — the
  wheels would go live the instant it topples, which is when your hands are
  near it).

- **Position hold tuning.** Stability window found empirically:

  | scale_d | vel_scale_stop | duration | result |
  |---|---|---|---|
  | 50 | 5 | 62 s | stable, sluggish. \|encError\| 28.7, pitch sd 2.00 |
  | 40 | 5 | 320 s | **stable, best so far.** \|encError\| 22.6, pitch sd 1.69 |
  | 35 | 3.5 | — | unstable |
  | 50 | 25 | 27 s | unstable, fell over |

  40/5 beat 50/5 on every metric with no trade-off — position accuracy AND
  pitch quality both improved, which says 50 was simply under-gained.
  **35 has never actually been tested**: the UI was set to 35 but the bot ran
  40 (the CSV preamble caught the mismatch). Next step is 35, verified at the
  bot rather than at the slider.

  The `encError` offset is 88% one-sided against a fixed target, and survives
  5 minutes — that is a standing bias (CoM or IMU zero), not wander. More gain
  shrinks it but will not remove it; that is a `theta_offset` trim question.

  Bounded on **both** sides, so it is not the P:D ratio. Suspected cause of the
  upper bound on damping: `enc_velocity` updates only every ~100 ms
  (`POS_VEL_PERIOD_MS`) and `vel_damp` is applied *after* the rate limiter, so
  it lands as a 10 Hz staircase in a 100 Hz loop. Weighting it harder makes each
  step a bigger kick. Untested idea: light low-pass on `enc_velocity` to raise
  that ceiling.

- **Zones are inert.** `zone_c = 500` with errors that never exceed ~95 ticks
  means `scale_d` is the only scale ever selected. `scale_a`/`b`/`c` are dead
  config. Thresholds need to match the real error scale before zone scheduling
  does anything — but the scale ordering (A=60, B=80, C=200, D=50) is not
  monotonic, so his intent for each zone needs establishing first.
