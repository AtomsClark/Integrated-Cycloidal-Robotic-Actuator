# Scripts

All scripts assume the [`odrive`](https://pypi.org/project/odrive/) Python package is installed and the ODESC V4.2 is reachable over USB.

```sh
pip install odrive
```

| Script | Purpose |
|---|---|
| `setup.py` | Apply the known-good config and run motor + encoder calibration. |
| `motion_tune.py` | Run a step sequence and drop into a REPL for live gain tweaking. |
| `characterize.py` | `--mode sweep` for no-load η / current draw across velocities; `--mode stall` for held-output torque tests against a known load. |
| `backlash.py` | Single-encoder backlash test — clamp the output, ramp current bidirectionally, infer slack from the engagement points where Iq spikes. |

## Recommended order

1. **Bench, unloaded** → `python setup.py`
2. **Bench, unloaded** → `python motion_tune.py` to walk gains up from defaults
3. **Bench, unloaded** → `python characterize.py --mode sweep` to log no-load efficiency curve
4. **Output clamped to lever arm** → `python characterize.py --mode stall --i-max 15` to measure transmission efficiency against a known mass × arm
5. **Output clamped solid** → `python backlash.py` (repeat at several output positions, average)
