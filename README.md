# SpiderBot

A six-legged walking robot (hexapod) driven by a Raspberry Pi and a PS4 controller.

The Pi reads the controller, runs inverse kinematics for each leg, and streams servo
angles over USB serial to a **Pimoroni Servo 2040**, which drives the robot's 18 hobby
servos (3 per leg). A nineteenth "pincer" servo hangs straight off a Pi GPIO pin.

```
 DualShock 4 ──Bluetooth──▶ Raspberry Pi ──USB serial──▶ Servo 2040 ──PWM──▶ 18 leg servos
                            RPI/stateMachine.py           Servo2040/micropython/main.py
                                   │
                                   └──GPIO 17 (PWM)──▶ pincer servo
```

**What it can do**

- Walk forward, backward, left and right with a tripod gait (left stick)
- "Flex": tilt and lean the body (right stick)
- Manually position any single leg for calibration and testing (Triangle)
- Raise / lower the body (X / Square) and open / close the pincer (R2 / L2)

## Repository layout

```
SpiderBot/
├── README.md                 ← you are here
├── requirements.txt          ← Python packages for the Raspberry Pi
├── LICENSE
├── .gitignore
├── docs/
│   ├── SETUP.md              ← step-by-step install, pairing, first run, troubleshooting
│   ├── HARDWARE.md           ← servo channel map, leg layout, GPIO pins, leg geometry
│   ├── CONTROLS.md           ← every PS4 button, and the state machine behind them
│   └── PROTOCOL.md           ← the serial packet format between the Pi and the Servo 2040
├── RPI/                      ← everything that runs on the Raspberry Pi (Python 3)
│   ├── README.md             ← what each file here does and how to run it
│   ├── stateMachine.py       ← MAIN PROGRAM: walking, flex, per-leg control, pincer
│   ├── jumpTest.py           ← 4-leg jump experiment (idle → crouch → jump)
│   ├── helperClasses/        ← shared classes imported by the two scripts above
│   │   ├── IKEngine.py       ← 3-joint leg inverse kinematics
│   │   ├── Leg.py            ← per-leg state (position, gait waypoints)
│   │   └── Point.py          ← small 3-D vector class
│   ├── tools/                ← standalone one-off checks; no shared imports
│   │   ├── IKtest.py         ← type x, y, z and get servo angles; runs on any computer
│   │   ├── GPIOTest.py       ← prints the state of a limit switch on GPIO 17
│   │   └── Pincer.py         ← sweeps the pincer servo on GPIO 17
│   └── legacy/               ← earlier iterations, kept for reference only
│       ├── Controller.py     ← 4-leg version with limit switches, before helperClasses existed
│       └── ControllerMac.py  ← single-leg version used during early development
└── Servo2040/                ← firmware for the Pimoroni Servo 2040 — flash ONE of these
    ├── README.md             ← how to flash the board
    ├── micropython/main.py   ← CURRENT: Pimoroni MicroPython + ServoCluster, all 18 servos
    └── circuitpython/code.py ← OLDER: CircuitPython + adafruit_motor, kept for reference
```

## Requirements

### Hardware

| Part | Notes |
|---|---|
| Raspberry Pi | Any model with Bluetooth and the 40-pin header, running Raspberry Pi OS |
| Pimoroni Servo 2040 | RP2040 servo driver with 18 servo headers; connects to the Pi over USB |
| 18 × hobby servos | Standard 0–180° servos, 3 per leg (coxa, femur, tibia) |
| Servo power supply | Sized for 18 servos, fed into the Servo 2040's power input. USB alone is not enough |
| Sony DualShock 4 | Paired to the Pi over Bluetooth |
| USB-A → USB-C cable | Pi ↔ Servo 2040 (carries the serial link) |
| Pincer servo (optional) | Signal on Pi GPIO 17, powered from 5 V, common ground |

Leg link lengths the IK assumes: coxa 43, femur 60, tibia 110 mm. See
[docs/HARDWARE.md](docs/HARDWARE.md) for the full servo channel map and pin table.

### Software

- **Raspberry Pi:** Python 3.9 or newer (developed on 3.13) plus the packages in
  [requirements.txt](requirements.txt): `pyPS4Controller`, `pyserial`, `RPi.GPIO`, `gpiozero`.
- **Servo 2040:** Pimoroni's MicroPython build for the Servo 2040 (for `micropython/main.py`),
  or CircuitPython + the `adafruit_motor` library (for `circuitpython/code.py`).

## Quick start

The long version, with troubleshooting, is in [docs/SETUP.md](docs/SETUP.md).

**1. Flash the Servo 2040** (one time). See [Servo2040/README.md](Servo2040/README.md).
In short: install Pimoroni's MicroPython on the board, then copy
`Servo2040/micropython/main.py` to the board's root as `main.py`.

**2. Set up the Pi**

```bash
sudo apt update && sudo apt install -y git python3-venv python3-pip
git clone https://github.com/hans-vador/SpiderBot.git
cd SpiderBot
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
sudo usermod -aG dialout,input $USER   # serial + joystick access; log out and back in afterwards
```

**3. Pair the controller** (hold **Share + PS** until the light bar flashes, then):

```bash
bluetoothctl
# inside bluetoothctl:
#   scan on            → note the MAC of "Wireless Controller"
#   pair XX:XX:XX:XX:XX:XX
#   trust XX:XX:XX:XX:XX:XX
#   connect XX:XX:XX:XX:XX:XX
#   exit
ls /dev/input/js*       # should show /dev/input/js0
```

**4. Plug the Servo 2040 into the Pi over USB** and check it shows up:

```bash
ls /dev/ttyACM*         # should show /dev/ttyACM0
```

**5. Run it.** Run the main program by path. It imports `helperClasses` from the folder it
lives in, so keep `RPI/helperClasses/` next to it.

```bash
cd RPI
python3 stateMachine.py
```

> **Safety:** on start-up every leg immediately moves to the idle stance. Hold the robot
> off the ground, keep fingers clear of the joints, and power the servos only after the
> Servo 2040 has booted.

## Controls

Only the essentials; the full map and the state machine are in [docs/CONTROLS.md](docs/CONTROLS.md).
Note that the **legs** are named L1–L3 / R1–R3 and the **shoulder buttons** are also called L1 / R1.

| Input | Action |
|---|---|
| Left stick | Walk forward / backward / left / right |
| Right stick | Flex: tilt the body forward-back and side-to-side |
| Triangle | Toggle single-leg control mode |
| L1 / R1 (in single-leg mode) | Select previous / next leg |
| Left stick + D-pad ↑↓ (in single-leg mode) | Move the selected foot in x / z and y |
| D-pad ← / → | Speed down / up (0.5 – 1.5) |
| X / Square | Lower every foot / raise every foot by 70 (body goes up / down) |
| Circle | Reset all legs to the idle stance |
| R2 / L2 | Pincer to ~0° / ~180° |

## How it works

1. `pyPS4Controller` calls a method on `MyController` for every button and stick event.
   Stick, D-pad and Triangle handlers only record the input. Circle, Square, X, L2 and R2
   act immediately from inside the handler.
2. A background thread in `stateMachine.py` runs a state machine (`idle`, `walkFoward`,
   `walkBackward`, `walkLeft`, `walkRight`, `flex`, `controlLeg`) about 16 times a second.
   Each tick it updates a target `Point(x, y, z)` for each foot.
3. `IKEngine.calculate()` turns a foot position into three servo angles using the
   femur and tibia lengths (`COXA` is defined but unused). Right-side legs are mirrored.
4. The angles are sent as one line of text per leg, e.g. `LEG:L1,S1:90.0,S2:48.3,S3:50.8`,
   over `/dev/ttyACM0` at 115200 baud. Identical consecutive lines are skipped.
5. `main.py` on the Servo 2040 parses each line, clamps the angles to 0–180 and writes
   them to the three servos mapped to that leg. Anything it prints comes back to the Pi
   and is echoed as `Servo2040 -> …`.

Walking is a tripod gait: legs L1, L3, R2 and legs R1, R3, L2 move half a cycle apart,
each foot stepping through end → lower-mid → start → upper-mid (lifted) waypoints.

## Other scripts

| Script | Runs on | What it does |
|---|---|---|
| `RPI/jumpTest.py` | Pi | Four-leg experiment: Square = idle stance, Circle = crouch, X = jump. Run from inside `RPI/` |
| `RPI/tools/IKtest.py` | Any computer | Interactive IK calculator, no hardware needed: `python3 RPI/tools/IKtest.py` |
| `RPI/tools/GPIOTest.py` | Pi | Prints PRESSED / RELEASED for a normally-open switch on GPIO 17 |
| `RPI/tools/Pincer.py` | Pi | Sweeps the pincer servo on GPIO 17 from ~0° to ~180° and stops |
| `RPI/legacy/*.py` | Pi | Older controllers kept for reference; `Controller.py` still runs but only drives 4 legs |

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError: No module named 'helperClasses'` | `RPI/helperClasses/` is no longer next to the script. Run it in place: `python3 RPI/stateMachine.py` |
| `PermissionError: /dev/ttyACM0` | Add yourself to the `dialout` group and log back in |
| `FileNotFoundError: /dev/ttyACM0` | The Servo 2040 is not plugged in, or is in bootloader mode (shows as an `RPI-RP2` drive) |
| `FileNotFoundError: /dev/input/js0` | Controller is not connected; re-run the `bluetoothctl` steps |
| `RuntimeError` from `import RPi.GPIO` | On a Pi 5 / new kernel: `pip uninstall RPi.GPIO && pip install rpi-lgpio` |
| `error: externally-managed-environment` from pip | You are not inside the venv; run `source venv/bin/activate` first |
| Servos twitch or the board resets | Servo power supply is too weak; do not power 18 servos from USB |

## License

MIT — see [LICENSE](LICENSE).
