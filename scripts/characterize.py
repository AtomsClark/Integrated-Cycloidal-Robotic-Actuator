"""
Efficiency + torque-output characterization for the cycloidal actuator.

Two test modes; pick one with --mode.

--mode sweep
    No-load velocity sweep at fixed current cap. Logs:
        commanded vel, measured vel, Vbus, Iq, Ibus
    For each setpoint, computes
        electrical_in   = Vbus * Ibus
        mechanical_out_input_side = (Iq * Kt) * (vel * 2*pi)        [N·m·rad/s = W]
        mechanical_out_output_side = mechanical_out_input_side / GEAR_RATIO
                                     (assumes lossless reduction — i.e., this is
                                      the *upper bound* on output power)
    Useful for measuring electrical efficiency and idling losses, and for
    finding the velocity above which iron/friction losses dominate.

--mode stall
    Held-output torque test. CLAMP THE OUTPUT to a lever arm of known length L
    with a hanging mass m at the end. Setpoint ramps current from 0 to
    a configurable ceiling. Script logs Iq, Ibus, Vbus while you watch when
    the output starts to lift the mass. Pass the lift-off Iq with --kt to
    compute measured output torque vs theoretical (Iq * Kt * GEAR_RATIO),
    giving you a transmission efficiency number that doesn't assume a
    lossless reduction.

Both modes write a CSV next to this script.

Usage:
    python scripts/characterize.py --mode sweep
    python scripts/characterize.py --mode stall --i-max 15
"""

import argparse
import csv
import math
import time

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
)

CPR = 8192
GEAR_RATIO = 19.0
KV = 90
KT = 8.27 / KV  # N·m / A


def connect():
    odrv = odrive.find_any()
    ax = odrv.axis0
    return odrv, ax


def sweep(odrv, ax, vels, dwell_s, out_path):
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.input_vel = 0
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    rows = []
    for v in vels:
        ax.controller.input_vel = v
        # let things settle
        time.sleep(dwell_s)
        # average over a window
        samples = []
        t_end = time.time() + 0.5
        while time.time() < t_end:
            samples.append({
                "t": time.time(),
                "vel_set": v,
                "vel": ax.encoder.vel_estimate,
                "Iq": ax.motor.current_control.Iq_measured,
                "Ibus": odrv.ibus,
                "Vbus": odrv.vbus_voltage,
            })
            time.sleep(0.005)

        avg = lambda k: sum(s[k] for s in samples) / len(samples)
        v_mean = avg("vel")          # counts/sec on input shaft
        Iq = avg("Iq")
        Ibus = avg("Ibus")
        Vbus = avg("Vbus")

        omega_in = (v_mean / CPR) * 2 * math.pi    # rad/s on input shaft
        omega_out = omega_in / GEAR_RATIO          # rad/s on output shaft
        tau_motor = Iq * KT                        # N·m on input shaft
        p_mech_in = abs(tau_motor * omega_in)
        p_mech_out_max = p_mech_in                  # upper bound (lossless gearbox)
        p_elec = abs(Vbus * Ibus)
        eta_elec = (p_mech_in / p_elec) if p_elec > 1e-3 else 0.0

        rows.append({
            "vel_set_cps": v,
            "vel_meas_cps": v_mean,
            "omega_in_rad_s": omega_in,
            "omega_out_rad_s": omega_out,
            "Iq_A": Iq,
            "Ibus_A": Ibus,
            "Vbus_V": Vbus,
            "tau_motor_Nm": tau_motor,
            "p_elec_W": p_elec,
            "p_mech_W": p_mech_in,
            "eta_electrical": eta_elec,
        })
        print(f"  v={v:>6.0f}  Iq={Iq:+.2f}A  Ibus={Ibus:+.2f}A  "
              f"P_in={p_elec:5.1f}W  P_mech={p_mech_in:5.1f}W  "
              f"η_elec={eta_elec*100:4.1f}%")

    ax.controller.input_vel = 0
    ax.requested_state = AXIS_STATE_IDLE

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nSweep log: {out_path}")


def stall(odrv, ax, i_max, ramp_s, out_path):
    """
    Output must be clamped to a lever arm with a known load.
    The script ramps torque setpoint from 0 to KT*i_max.
    Operator records the Iq at which the load lifts off.
    """
    ax.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    ax.controller.input_torque = 0
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    print("Stall test starting in 3 s — ensure output is clamped to lever arm.")
    time.sleep(3)

    rows = []
    n = max(1, int(ramp_s / 0.05))
    for i in range(n + 1):
        frac = i / n
        target_torque_motor = frac * (i_max * KT)
        ax.controller.input_torque = target_torque_motor
        time.sleep(0.05)
        rows.append({
            "t": time.time(),
            "torque_set_motor_Nm": target_torque_motor,
            "Iq_A": ax.motor.current_control.Iq_measured,
            "Ibus_A": odrv.ibus,
            "Vbus_V": odrv.vbus_voltage,
            "shadow": ax.encoder.shadow_count,
            "vel": ax.encoder.vel_estimate,
        })
        print(f"  τ_set={target_torque_motor:.3f} Nm motor "
              f"({target_torque_motor*GEAR_RATIO:.2f} Nm out, lossless)  "
              f"Iq={rows[-1]['Iq_A']:+.2f}A  vel={rows[-1]['vel']:+.0f}cps")

    ax.controller.input_torque = 0
    ax.requested_state = AXIS_STATE_IDLE

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nStall log: {out_path}")
    print("Lift-off torque (output) = Iq_at_liftoff * Kt * GEAR_RATIO * η_mech.")
    print("Compare to m * g * L (mass * gravity * lever-arm length) for η_mech.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=("sweep", "stall"), required=True)
    p.add_argument("--i-max", type=float, default=15.0,
                   help="stall: max motor current (A) at end of ramp")
    p.add_argument("--ramp-s", type=float, default=10.0,
                   help="stall: ramp duration (s)")
    p.add_argument("--vmin", type=float, default=2000)
    p.add_argument("--vmax", type=float, default=30000)
    p.add_argument("--vsteps", type=int, default=8)
    p.add_argument("--dwell", type=float, default=1.0,
                   help="sweep: settle time per setpoint (s)")
    args = p.parse_args()

    odrv, ax = connect()
    try:
        if args.mode == "sweep":
            step = (args.vmax - args.vmin) / max(1, args.vsteps - 1)
            vels = [args.vmin + step * i for i in range(args.vsteps)]
            vels = [+v for v in vels] + [-v for v in vels]
            sweep(odrv, ax, vels, args.dwell, "sweep_log.csv")
        else:
            stall(odrv, ax, args.i_max, args.ramp_s, "stall_log.csv")
    finally:
        ax.requested_state = AXIS_STATE_IDLE


if __name__ == "__main__":
    main()
