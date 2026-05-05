# Flipsky ODESC V4.2 — `odrivetool` setup transcript

Verbatim transcription of the working notes from `FLIPSKY_ODESC_Programming.pages` (2026-02-24). Captures the iteration sequence used to get an Eaglepower 8308 turning under closed-loop position control on 12 V.

```
FLIPSKY ODESC V4.2 24V
FW 0.4.12
12V 30A PSU

odrivetool

odrv0.erase_configuration()

odrv0.config.brake_resistance = 2
odrv0.config.dc_max_positive_current = 30
odrv0.config.dc_max_negative_current = -2.0
odrv0.config.max_regen_current = 0
odrv0.save_configuration()

odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_PMSM_CURRENT_CONTROL
odrv0.axis0.motor.config.pole_pairs = 20
odrv0.axis0.motor.config.torque_constant = 8.27/90
odrv0.axis0.motor.config.resistance_calib_max_voltage = 5
odrv0.axis0.motor.config.calibration_current = 2
odrv0.axis0.motor.config.current_lim = 22
odrv0.axis0.motor.config.requested_current_range = 30

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.motor
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.calib_range = 0.1
odrv0.save_configuration()

odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.encoder.config.cpr = 8192
odrv0.axis0.controller.config.pos_gain = 20
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.1
odrv0.axis0.controller.config.vel_limit = 100
odrv0.save_configuration()

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis0.encoder
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()
odrv0.reboot()


odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()
odrv0.reboot()

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis1.requested_state = AXIS_STATE_IDLE
odrv0.save_configuration()


odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.reboot()

odrv0.axis0.controller.config.vel_limit = 50000
odrv0.axis0.controller.config.vel_limit_tolerance = 2.0

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.pos_setpoint = 8192

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_gain = 0.0001
odrv0.axis0.controller.config.vel_integrator_gain = 0.0001
odrv0.axis0.controller.config.pos_gain = 5
odrv0.axis0.controller.config.vel_ramp_rate = 2000
odrv0.axis0.controller.vel_ramp_enable = True
odrv0.axis0.motor.config.current_lim_tolerance = 5.0
odrv0.axis0.motor.config.current_control_bandwidth = 100
odrv0.save_configuration()


odrv0.axis0.encoder.shadow_count

# Set extremely low gains to prevent the initial "kick"
odrv0.axis0.controller.config.pos_gain = 1.5
odrv0.axis0.controller.config.vel_gain = 0.0001
odrv0.axis0.controller.config.vel_integrator_gain = 0.001

# Limit the speed so it doesn't overshoot at 12V
# 8192 counts/sec = 1 rotation per second
odrv0.axis0.controller.config.vel_limit = 10000

# Expand current headroom to avoid the violation trip
odrv0.axis0.motor.config.current_lim = 20.0
odrv0.axis0.motor.config.current_lim_tolerance = 10.0

odrv0.save_configuration()
odrv0.reboot()

odrv0.axis0.controller.pos_setpoint = 500
```

## Notes

- **Motor:** Eaglepower 8308, 90 Kv → `torque_constant = 8.27 / 90`, 20 pole pairs.
- **Encoder:** AMT103-V incremental, 8192 CPR, with index pulse used (`use_index = True`).
- **Calibration current:** 2 A, with `resistance_calib_max_voltage = 5 V`.
- **First-pass gains** that worked once but were too snappy: `pos_gain = 20`, `vel_gain = 0.02`, `vel_integrator_gain = 0.1`, `vel_limit = 100`.
- **Final low-kick gains** that survive the closed-loop transition at 12 V without tripping the `current_lim` violation: `pos_gain = 1.5`, `vel_gain = 0.0001`, `vel_integrator_gain = 0.001`, `vel_limit = 10000`, `current_lim = 20.0`, `current_lim_tolerance = 10.0`.
- **Velocity-mode tune:** `vel_gain = 0.0001`, `vel_integrator_gain = 0.0001`, `pos_gain = 5`, `vel_ramp_rate = 2000`, `vel_ramp_enable = True`, `current_control_bandwidth = 100`.
- **Test command:** `odrv0.axis0.controller.pos_setpoint = 500` (counts) for a small bench step once in `AXIS_STATE_CLOSED_LOOP_CONTROL`.
