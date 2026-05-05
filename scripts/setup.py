"""
Apply the known-good ODESC V4.2 configuration for the Eaglepower 8308 +
AMT103-V combo, run motor + encoder calibration, and persist.

Run with the actuator unloaded and free to spin. After completion the board
will reboot and come up in CLOSED_LOOP_CONTROL automatically.

Usage:
    python scripts/setup.py
"""

import time
import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    ENCODER_MODE_INCREMENTAL,
    MOTOR_TYPE_PMSM_CURRENT_CONTROL,
)


# Eaglepower 8308 + AMT103-V on Flipsky ODESC V4.2 @ 12 V bench supply.
EAGLEPOWER_8308_KV = 90
ENCODER_CPR = 8192


def wait_for_idle(axis, timeout_s=30):
    t0 = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - t0 > timeout_s:
            raise TimeoutError("axis did not return to IDLE")
        time.sleep(0.2)


def main():
    print("Searching for ODESC...")
    odrv = odrive.find_any()
    print(f"Found: {odrv.serial_number}, fw {odrv.fw_version_major}."
          f"{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    print("Erasing existing configuration...")
    try:
        odrv.erase_configuration()
    except Exception:
        # erase_configuration triggers a reboot — reconnect.
        pass
    time.sleep(2)
    odrv = odrive.find_any()

    # ---- power stage ----
    odrv.config.brake_resistance = 2
    odrv.config.dc_max_positive_current = 30
    odrv.config.dc_max_negative_current = -2.0
    odrv.config.max_regen_current = 0

    ax = odrv.axis0

    # ---- motor ----
    ax.motor.config.motor_type = MOTOR_TYPE_PMSM_CURRENT_CONTROL
    ax.motor.config.pole_pairs = 20
    ax.motor.config.torque_constant = 8.27 / EAGLEPOWER_8308_KV
    ax.motor.config.resistance_calib_max_voltage = 5
    ax.motor.config.calibration_current = 2
    ax.motor.config.current_lim = 20.0
    ax.motor.config.current_lim_tolerance = 10.0
    ax.motor.config.requested_current_range = 30
    ax.motor.config.current_control_bandwidth = 100

    # ---- encoder ----
    ax.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    ax.encoder.config.cpr = ENCODER_CPR
    ax.encoder.config.use_index = True
    ax.encoder.config.calib_range = 0.1

    # ---- controller (low-kick gains that survive 12 V closed-loop entry) ----
    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    ax.controller.config.pos_gain = 1.5
    ax.controller.config.vel_gain = 0.0001
    ax.controller.config.vel_integrator_gain = 0.001
    ax.controller.config.vel_limit = 10000  # 8192 cpr ~= 1 rev/s

    print("Saving pre-cal configuration...")
    odrv.save_configuration()
    time.sleep(2)
    odrv = odrive.find_any()
    ax = odrv.axis0

    # ---- motor calibration ----
    print("Running motor calibration (rotor will twitch)...")
    ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    wait_for_idle(ax)
    if ax.motor.error:
        raise RuntimeError(f"motor cal failed: error=0x{ax.motor.error:08x}")
    ax.motor.config.pre_calibrated = True

    # ---- encoder calibration (with index) ----
    print("Running full calibration sequence (rotor will spin to find index)...")
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    wait_for_idle(ax, timeout_s=60)
    if ax.encoder.error:
        raise RuntimeError(f"encoder cal failed: error=0x{ax.encoder.error:08x}")
    ax.encoder.config.pre_calibrated = True

    # ---- startup behavior ----
    ax.config.startup_closed_loop_control = True

    print("Saving calibrated configuration and rebooting...")
    odrv.save_configuration()
    try:
        odrv.reboot()
    except Exception:
        pass

    print("Setup complete. After reboot the axis will come up in"
          " CLOSED_LOOP_CONTROL with low-kick gains.")


if __name__ == "__main__":
    main()
