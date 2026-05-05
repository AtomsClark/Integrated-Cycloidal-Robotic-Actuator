"""
Simple motion + interactive gain tuning for the cycloidal actuator.

What it does:
    1. Connect, ensure the axis is in CLOSED_LOOP_CONTROL with the saved low-kick gains.
    2. Run a small bidirectional position-step sequence (1, 2, 4, 8 input revs)
       and log shadow_count, current, vel, and the position error vs setpoint.
    3. Drop into a REPL where you can poke gains live:
            pos = <int>           # absolute setpoint in counts
            rev <float>           # setpoint in input revolutions
            step <float>          # relative setpoint in input revolutions
            pg <float>            # set pos_gain
            vg <float>            # set vel_gain
            vig <float>           # set vel_integrator_gain
            cl <float>            # set current_lim
            log [seconds]         # log a window of state to step_log.csv
            home                  # set 0
            idle                  # AXIS_STATE_IDLE
            run                   # AXIS_STATE_CLOSED_LOOP_CONTROL
            q                     # quit

Tuning approach: increase pos_gain until the step response just starts to
overshoot, then back it off ~20%. Increase vel_integrator_gain to remove
steady-state error after that. vel_gain stays small — the gear ratio
already provides plenty of damping.

Usage:
    python scripts/motion_tune.py
"""

import csv
import math
import time
from pathlib import Path

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
)

CPR = 8192
GEAR_RATIO = 19.0  # input revs per output rev


def connect():
    print("Connecting...")
    odrv = odrive.find_any()
    ax = odrv.axis0
    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.5)
    return odrv, ax


def snapshot(ax, vbus):
    return {
        "t":        time.time(),
        "shadow":   ax.encoder.shadow_count,
        "pos_set":  ax.controller.pos_setpoint,
        "vel":      ax.encoder.vel_estimate,
        "iq":       ax.motor.current_control.Iq_measured,
        "vbus":     vbus,
    }


def log_window(odrv, ax, seconds, path):
    rows = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        rows.append(snapshot(ax, odrv.vbus_voltage))
        time.sleep(0.005)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"  wrote {len(rows)} rows to {path}")


def step_sequence(odrv, ax):
    print("Bidirectional step sequence: 1, 2, 4, 8 input revs.")
    home = ax.encoder.shadow_count
    rows = []
    for revs in (1, 2, 4, 8):
        for direction in (+1, -1):
            target = home + direction * revs * CPR
            ax.controller.pos_setpoint = target
            t_end = time.time() + max(1.0, revs * 0.5)
            while time.time() < t_end:
                rows.append(snapshot(ax, odrv.vbus_voltage))
                time.sleep(0.005)
            ax.controller.pos_setpoint = home
            t_end = time.time() + max(1.0, revs * 0.5)
            while time.time() < t_end:
                rows.append(snapshot(ax, odrv.vbus_voltage))
                time.sleep(0.005)

    out = Path("step_log.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"Step sequence done. Log: {out}")


def repl(odrv, ax):
    print("\nInteractive REPL. Type 'q' to quit.")
    print(f"  pos_gain={ax.controller.config.pos_gain}"
          f"  vel_gain={ax.controller.config.vel_gain}"
          f"  vel_integrator_gain={ax.controller.config.vel_integrator_gain}"
          f"  current_lim={ax.motor.config.current_lim}")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        try:
            tok = cmd.split()
            head = tok[0]
            if head == "q":
                break
            elif head == "pos":
                ax.controller.pos_setpoint = int(tok[1])
            elif head == "rev":
                ax.controller.pos_setpoint = int(float(tok[1]) * CPR)
            elif head == "step":
                ax.controller.pos_setpoint = (
                    ax.encoder.shadow_count + int(float(tok[1]) * CPR)
                )
            elif head == "pg":
                ax.controller.config.pos_gain = float(tok[1])
            elif head == "vg":
                ax.controller.config.vel_gain = float(tok[1])
            elif head == "vig":
                ax.controller.config.vel_integrator_gain = float(tok[1])
            elif head == "cl":
                ax.motor.config.current_lim = float(tok[1])
            elif head == "log":
                seconds = float(tok[1]) if len(tok) > 1 else 2.0
                log_window(odrv, ax, seconds, "live_log.csv")
            elif head == "home":
                ax.controller.pos_setpoint = ax.encoder.shadow_count
            elif head == "idle":
                ax.requested_state = AXIS_STATE_IDLE
            elif head == "run":
                ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            else:
                print("?")
                continue
            print(f"  shadow={ax.encoder.shadow_count}  "
                  f"vel={ax.encoder.vel_estimate:.0f}  "
                  f"Iq={ax.motor.current_control.Iq_measured:+.2f}A")
        except Exception as e:
            print(f"  error: {e}")


def main():
    odrv, ax = connect()
    try:
        step_sequence(odrv, ax)
        repl(odrv, ax)
    finally:
        ax.requested_state = AXIS_STATE_IDLE
        print("Axis idled.")


if __name__ == "__main__":
    main()
