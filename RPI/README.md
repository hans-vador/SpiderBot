# RPI/ — the Raspberry Pi side

Python 3 code that runs on the Pi. Full install steps are in [../docs/SETUP.md](../docs/SETUP.md);
the packages it needs are in [../requirements.txt](../requirements.txt).

`stateMachine.py` and `jumpTest.py` import `helperClasses` from this folder, so keep
`helperClasses/` next to them and run the scripts by path:

```bash
cd RPI
python3 stateMachine.py
```

| Path | What it is |
|---|---|
| `stateMachine.py` | Main program: PS4 controller → state machine → IK → serial packets to the Servo 2040 |
| `jumpTest.py` | Four-leg jump experiment (Square = idle, Circle = crouch, X = jump) |
| `helperClasses/Point.py` | 3-D vector with `+ - * /`, `distance_to`, `midpoint` |
| `helperClasses/Leg.py` | Per-leg state: current position, target, gait waypoints |
| `helperClasses/IKEngine.py` | Foot position → (S1, S2, S3) servo angles; right legs mirrored |
| `tools/IKtest.py` | Interactive IK calculator, no hardware, runs anywhere |
| `tools/GPIOTest.py` | Prints a limit switch on GPIO 17 |
| `tools/Pincer.py` | Sweeps the pincer servo on GPIO 17 |
| `legacy/Controller.py` | Earlier 4-leg controller with limit switches, everything in one file |
| `legacy/ControllerMac.py` | Earlier single-leg dev version |

Controls: [../docs/CONTROLS.md](../docs/CONTROLS.md). Pins and servo map: [../docs/HARDWARE.md](../docs/HARDWARE.md).
