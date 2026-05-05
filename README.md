# BLDC Integrated Cycloidal Actuator

A compact, high-torque robot joint actuator integrating an Eaglepower 8308 BLDC motor with a 3D-printed 19:1 cycloidal reduction. Designed and built January 2026 as a candidate joint for a humanoid robot.

A writeup of this project lives on my portfolio: <https://atomsclark.com/projects/cycloidal-actuator>. This repo holds the full source: CAD, photos, firmware artifacts, and tuning notes.

## Specs at a glance

| | |
|---|---|
| Reduction | 19:1 single-stage cycloidal |
| Cycloidal disc lobes | 19 (paired discs, 180° out of phase) |
| Ring-gear pins | 20, integrated to ring body |
| Output pins | 6 |
| Final torque | ~50 Nm |
| Efficiency | ~70% |
| Backlash | ~2° |
| Backdrivability | poor |
| Motor | Eaglepower 8308 BLDC |
| Controller | Flipsky ODESC V4.2 24V |
| Encoder | CUI Devices AMT103-V (incremental, 8192 CPR) |
| Mass | 3D-printed PETG / PLA on a Bambu P1P |
| Lubrication | white lithium grease |

## Repo layout

```
cad/
  v1.0/                  STLs + .3mf for the built version (Onshape source)
  archive/               Earlier iterations (V0, V0.1, V0.2) and disc source files
media/
  photos/                Real-world photos of parts and assembly
  cad-renders/           Onshape renders incl. cross-sections
firmware/
  FLIPSKY_ODESC_Programming.pages   Tuning notes from getting the controller running
  Flipsky_EDC_Default_Firmware.bin  Stock controller firmware (backup)
  ODriveFirmware_v3.6-24V_0.4.12.elf  ODrive 0.4.12 firmware that the ODESC matches
docs/
  CapstonePhase1_RobotRecordFlipper.key   Original concept slides
  MotorSizing.numbers                     Motor selection math
  CapstoneProject_BoM.numbers             Original BoM
```

## Bill of Materials

| Item | Qty | Cost |
|---|---:|---:|
| Eaglepower 8308 BLDC motor | 1 | $80 |
| Flipsky ODESC V4.2 24V FOC controller ([B0CB64MVHC](https://www.amazon.com/dp/B0CB64MVHC)) | 1 | $40 |
| CUI AMT103-V incremental encoder | 1 | $30 |
| 3×7×3 mm bearing — output pins ([B0DXZKVZMR](https://www.amazon.com/dp/B0DXZKVZMR)) | 12 | $15 |
| 6805-2RS bearing, 25×37×7 mm ([B082PYT33D](https://www.amazon.com/dp/B082PYT33D)) | 2 | $15 |
| 6806-2RS bearing, 30×42×7 mm ([B082PXK5K9](https://www.amazon.com/dp/B082PXK5K9)) | 2 | $15 |
| 6816-2RS bearing, 80×100×10 mm ([B07RQ4RXDR](https://www.amazon.com/dp/B07RQ4RXDR)) | 2 | $15 |
| M3 hardware + brass heat-set inserts | — | ~$25 |
| PETG + PLA filament | — | ~$15 |
| White lithium grease | — | ~$5 |
| **Total** | | **~$255** |

## Cycloidal Disc Geometry

Disc profiles started from STP files generated on [mevirtuoso.com/cycloidal-drive](https://mevirtuoso.com/cycloidal-drive/). I used 19 lobes on the disc paired with 20 ring-gear pins (N+1 convention) for the 19:1 reduction. Six output pins drive the common output disc.

## Print Settings

- Printer: Bambu P1P
- Materials: PETG (structural) and PLA (non-load-bearing parts)
- Heat-set inserts (M3 brass) for every fastener interface so the actuator can be reassembled without stripping plastic threads

## Controller Setup (Flipsky ODESC V4.2 → Eaglepower 8308)

The ODESC V4.2 advertises ODrive compatibility but ships with abandoned/forked firmware. Most of the integration time went into figuring out which `odrivetool` flags actually work against it and which gains keep the closed-loop transition from tripping a current-limit violation.

The full bench transcript (every command run, in order, including the iteration history) lives in [`firmware/odrivetool-session.md`](firmware/odrivetool-session.md). The minimal "what worked" sequence is below.

**Hardware**
- Flipsky ODESC V4.2 24V, ODrive-compatible firmware 0.4.12 (recovery image: `firmware/Flipsky_EDC_Default_Firmware.bin`; matching ODrive ELF: `firmware/ODriveFirmware_v3.6-24V_0.4.12.elf`)
- 12 V / 30 A bench PSU
- Eaglepower 8308 (90 Kv, 20 pole pairs)
- AMT103-V incremental encoder, 8192 CPR, index pulse used

**Power stage**
```python
odrv0.erase_configuration()
odrv0.config.brake_resistance        = 2
odrv0.config.dc_max_positive_current = 30
odrv0.config.dc_max_negative_current = -2.0
odrv0.config.max_regen_current       = 0
odrv0.save_configuration()
```

**Motor calibration**
```python
odrv0.axis0.motor.config.motor_type                   = MOTOR_TYPE_PMSM_CURRENT_CONTROL
odrv0.axis0.motor.config.pole_pairs                   = 20
odrv0.axis0.motor.config.torque_constant              = 8.27 / 90   # Kv = 90
odrv0.axis0.motor.config.resistance_calib_max_voltage = 5
odrv0.axis0.motor.config.calibration_current          = 2
odrv0.axis0.motor.config.current_lim                  = 22
odrv0.axis0.motor.config.requested_current_range      = 30

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.save_configuration()
```

**Encoder**
```python
odrv0.axis0.encoder.config.mode      = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr       = 8192
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.save_configuration()
```

**Controller — final low-kick gains**
```python
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

# Walk pos/vel gains down from ODrive defaults to avoid the initial
# "kick" tripping current_lim_violation when entering closed loop at 12 V.
odrv0.axis0.controller.config.pos_gain            = 1.5
odrv0.axis0.controller.config.vel_gain            = 0.0001
odrv0.axis0.controller.config.vel_integrator_gain = 0.001

# 8192 counts/sec = 1 rev/sec — cap velocity to prevent overshoot at 12 V
odrv0.axis0.controller.config.vel_limit = 10000

# Wider current headroom so transients don't trip the violation
odrv0.axis0.motor.config.current_lim           = 20.0
odrv0.axis0.motor.config.current_lim_tolerance = 10.0

odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()
odrv0.reboot()
```

**Gotchas hit**
- ODrive's stock `pos_gain = 20` blew the current-limit violation as soon as `AXIS_STATE_CLOSED_LOOP_CONTROL` engaged — `pos_gain` had to come down to ~1.5 with `vel_gain = 0.0001` before the transient settled cleanly at 12 V.
- `current_lim_tolerance = 10.0` is generous, but anything tighter and the violation tripped on every encoder index search.
- Several newer ODrive config knobs silently no-op on the ODESC firmware — stick to the 0.4.12 surface area.

## Why this didn't ship in the humanoid

- **2° backlash** — printed cycloidal geometry has a hard floor on backlash without post-machining.
- **Poor backdrivability** — at 19:1 with printed friction surfaces, the joint can't be backdriven by hand, which kills compliant control schemes.
- **Cost per joint (~$255) and print time** — across all the DOFs I wanted, the BoM and assembly time put a full humanoid out of hobby budget.

So this build is now a self-contained study in BLDC + cycloidal joint design, and the humanoid moved to a different actuation strategy.

## License

Personal project — no license set. Ask before reusing.
