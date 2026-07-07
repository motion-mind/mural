# Mural fork: original hardware only (no wiring changes)

## What's included

1. **Binary job format** (replacing the old text-line `/commands` file),
   format version 3
2. **Fixed-point int32 coordinates** (replacing float32) - exact decimal
   values, no IEEE-754 rounding
3. **Display removed** (SSD1306 OLED and all code referencing it) - the only
   behavior change worth knowing: the OLED was the sole way the original
   firmware showed the device's IP on first boot. Replaced with a
   `Serial.println()` of the same info at the end of `setup()`.
   `http://mural.local` (mDNS) still works too, unaffected either way.
4. **Or-opt travel-distance reduction**, layered on top of the existing
   greedy nearest-neighbor path ordering (`tsc/src/optimizer.ts`, untouched)
5. **Velocity/acceleration profiling** - per-move target speed instead of one
   hardcoded constant for every move in a job
6. **Pen up/down timing: fixed-position calibration + always-full-swing settle time
   
## Format reference (version 3)

```
Header (9 bytes):
  offset 0: uint8   formatVersion   (3)
  offset 1: float32 totalDistance   (mm)
  offset 5: float32 height          (mm, informational only)

Records (repeated until EOF):
  0x00                                pen up    -> 1 byte
  0x01                                pen down  -> 1 byte
  0x02, int32 x, int32 y, uint16 speed   move   -> 11 bytes
        (x, y: fixed-point, COORDINATE_SCALE=100 -> 0.01mm/unit)
        (speed: fixed-point, SPEED_SCALE=10 -> 0.1mm/s/unit)
```

## Velocity profiling - the corrected design, and why it changed

`tsc/src/velocityPlanner.ts` computes a per-point target speed across the
*entire job as one continuous polyline* - travel moves included, not just
pen-down strokes - ramping up from a standing start, slowing for sharp
corners, ramping down to a stop at the true end. It never reorders, adds,
removes, or moves any point, only annotates a speed onto each one.

**Constants** (`tsc/src/velocityPlanner.ts`): `MAX_SPEED_MM_PER_SEC = 15` is
set close to what the old `printSpeedSteps` constant already implied in mm/s,
so cruise speed on long straight runs is roughly unchanged from before.
`MAX_ACCEL_MM_PER_SEC2 = 100` has no old equivalent to anchor to - the
previous firmware accelerated "instantly" - so it's a genuine guess pending
hardware tuning. `CORNER_FACTOR_POWER = 2` controls cornering aggressiveness,
also untuned.

**Firmware side:** `beginLinearTravel`'s speed parameter changed from
steps/sec to mm/s. It now captures the pen's previous position before
overwriting it, computes the actual straight-line distance for this hop, and
derives both motors' step rates from `moveTime = distance / speed` directly -
which also let me delete the old `if (deltaLeft >= deltaRight) {...} else
{...}` branch entirely, since it was only there to handle a raw-steps input.
`defaultSpeedMmPerSec` / `homeReturnSpeedMmPerSec` in `movement.h` are
derived (not re-measured) mm/s equivalents of the old
`printSpeedSteps`/`moveSpeedSteps` constants, so the one non-planned call
site (the end-of-job return-to-home move) behaves the same as before.
`MAX_ALLOWED_SPEED_MM_PER_SEC` is a defensive clamp independent of whatever
the browser sends.

---

## Pen up/down timing: fixed-position calibration + always-full-swing settle time

This supersedes the distance-proportional settle-time fix from earlier in this
thread. That version computed settle time from the actual angular distance
between wherever the pen currently was and its target - which meant it
depended on `currentPosition`/`penDistance` bookkeeping being accurate, and a
user-calibrated "down" angle that could vary session to session. The current
version removes both sources of variability instead of just estimating
around them.

**Calibration is now fixed-servo, adjustable-pen instead of the other way
around.** Previously: the pen mechanism was fixed close to the wall by hand,
then the *servo angle* was fine-tuned via a web UI slider until the pen just
touched the surface, and that arbitrary angle got stored as `penDistance`.
Now: `PEN_DOWN_DEGREES` in `pen.h` is a fixed constant (full servo extension).
Entering the pen calibration phase moves the servo there immediately -
`PhaseManager::setPhase()` calls the new `PenCalibrationPhase::onEnter()`,
which calls `pen->slowDown()` - and the person calibrating physically loosens
the pen holder and slides the *pen* until it just touches the wall at that
fixed angle, then tightens the bolt. The web UI's slider and +/- buttons are
gone; calibration is now a single "Pen is touching the wall" confirm button
with no parameters (`/setPenDistance` renamed to `/confirmPenCalibration`
throughout - route, `Phase` virtual method, and the one frontend AJAX call).

**Timing always assumes a full swing, because now it genuinely always is
one.** Since `PEN_DOWN_DEGREES` is fixed, every real pen transition covers
exactly `PEN_UP_DEGREES - PEN_DOWN_DEGREES` (90°) - there's no more "assume
the worst case as a safety margin" reasoning, because there's no more
variation to be a worst case *of*. `PEN_SETTLE_MS` in `pen.h` is computed
once from that fixed 90° swing and the servo's derated effective speed, and
`doSlowMove()` in `pen.cpp` just uses that constant directly instead of
calculating anything per-move. This is both simpler (one less runtime
calculation, one less thing that could be wrong) and more reliable (doesn't
depend on any position bookkeeping being correct - if `currentPosition` were
ever somehow stale, the fixed settle time still applies unconditionally).

Same constants as before, same caveat: `SERVO_RATED_DEG_PER_SEC` from your
servo's actual spec (60°/0.11s), `SERVO_LOAD_DERATING = 0.5` still a genuine
guess at real-world load versus the no-load datasheet figure - the first
thing to adjust if timing still looks off.
