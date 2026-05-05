"""
Single-encoder backlash test for the cycloidal actuator.

Idea
----
With only an input-side encoder you can't directly measure output position,
but you *can* measure the moment the gearbox starts loading the motor.
Inside the backlash zone, the input shaft turns freely (current ≈ 0).
The instant the lobes engage, current spikes.

So: rigidly clamp the output, command a slow position ramp on the input,
log Iq vs shadow_count, and look for the steep rise. Reverse direction and
do it again. The angular distance between the two engagement points,
divided by the gear ratio, is the output-referred backlash.

Procedure
---------
1. Clamp the output disc to the desk fixture so it cannot rotate.
2. Run this script. It will:
     a. Drive +0.5 motor revs at low gain to find the +engagement point
        (Iq crosses --i-thresh).
     b. Reverse and drive -1.0 motor revs to find the -engagement point.
     c. Compute Δshadow between engagement points → input-shaft slack.
     d. Output backlash (deg) = (Δshadow / CPR) * 360 / GEAR_RATIO.
3. Repeat at multiple output positions (re-clamp, re-run). Backlash
   typically varies a bit around the rotation because of disc geometry
   tolerances; report mean ± range.

Tips
----
- Run on a stiff bench; any compliance in the clamp shows up as backlash.
- Use a small --i-thresh (e.g. 0.5 A) for sensitivity, but high enough to
  reject Iq noise.
- This measures total system slack: ring-pin clearance + disc-bore
  clearance + output-pin slop + any flex in the printed parts. That's
  fine — it's also what a downstream controller experiences.

Usage:
    python scripts/backlash.py --i-thresh 0.5 --speed 5000
"""

import argparse
import csv
import math
import time

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
)

CPR = 8192
GEAR_RATIO = 19.0


def connect():
    odrv = odrive.find_any()
    ax = odrv.axis0
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.input_vel = 0
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    return odrv, ax


def find_engagement(ax, direction, speed, i_thresh, max_revs, log):
    """
    Drive at constant velocity until |Iq| crosses i_thresh.
    Returns the shadow_count at the moment of engagement, or None if
    we ran the full max_revs without seeing engagement.
    """
    start = ax.encoder.shadow_count
    limit = start + direction * int(max_revs * CPR)

    ax.controller.input_vel = direction * speed
    engagement = None
    while True:
        s = ax.encoder.shadow_count
        iq = ax.motor.current_control.Iq_measured
        log.append({
            "t": time.time(),
            "shadow": s,
            "Iq": iq,
            "vel_set": direction * speed,
        })
        if abs(iq) > i_thresh:
            engagement = s
            break
        if (direction > 0 and s >= limit) or (direction < 0 and s <= limit):
            break
        time.sleep(0.002)

    ax.controller.input_vel = 0
    time.sleep(0.2)
    return engagement


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--i-thresh", type=float, default=0.5,
                   help="Iq (A) threshold that marks engagement")
    p.add_argument("--speed", type=float, default=5000,
                   help="counts/sec while searching")
    p.add_argument("--max-revs", type=float, default=0.5,
                   help="abort search after this many motor revs in one direction")
    args = p.parse_args()

    odrv, ax = connect()
    log = []
    try:
        print(f"Searching for + engagement at vel={args.speed} cps "
              f"(threshold {args.i_thresh} A)...")
        s_plus = find_engagement(ax, +1, args.speed, args.i_thresh,
                                 args.max_revs, log)
        if s_plus is None:
            raise RuntimeError("no + engagement found — output not clamped?")
        print(f"  + engagement at shadow={s_plus}")

        print("Reversing to find - engagement...")
        s_minus = find_engagement(ax, -1, args.speed, args.i_thresh,
                                  args.max_revs * 2, log)
        if s_minus is None:
            raise RuntimeError("no - engagement found")
        print(f"  - engagement at shadow={s_minus}")

        delta_counts = abs(s_plus - s_minus)
        input_deg = (delta_counts / CPR) * 360.0
        output_deg = input_deg / GEAR_RATIO
        print(f"\nInput-shaft slack:  {delta_counts} counts ({input_deg:.2f}°)")
        print(f"Output backlash:    {output_deg:.3f}°"
              f"  (over {GEAR_RATIO}:1 reduction)")
    finally:
        ax.requested_state = AXIS_STATE_IDLE
        with open("backlash_log.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(log[0].keys()))
            w.writeheader()
            w.writerows(log)
        print("Trace: backlash_log.csv")


if __name__ == "__main__":
    main()
