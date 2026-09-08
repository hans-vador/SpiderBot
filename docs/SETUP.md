# Setup guide

Everything you need to go from a fresh clone to a walking robot. The short version is in
the root [README](../README.md); this page spells out each step and what to do when it fails.

There are two computers to set up:

1. the **Servo 2040** board, which needs firmware flashed once, and
2. the **Raspberry Pi**, which runs the Python controller.

## 1. Servo 2040 firmware

Follow [Servo2040/README.md](../Servo2040/README.md). You end up with
`Servo2040/micropython/main.py` sitting on the board as `main.py`, so it starts
automatically every time the board powers up. Once flashed you can leave the board alone.

## 2. Raspberry Pi

### 2.1 Operating system and packages

Any Raspberry Pi OS (Bookworm or newer, Lite is fine) on a Pi that has Bluetooth.

```bash
sudo apt update
sudo apt install -y git python3-venv python3-pip
```

### 2.2 Clone the repo and create a virtual environment

```bash
git clone https://github.com/hans-vador/SpiderBot.git
cd SpiderBot
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

Why `--system-site-packages`: Raspberry Pi OS already ships `RPi.GPIO` and `gpiozero`
through apt. Creating the venv this way lets pip see them instead of trying to build
them from source. Recent Raspberry Pi OS refuses `pip install` outside a venv
(`externally-managed-environment`), so always activate the venv before running anything.

What gets installed, and who uses it:

| Package | Used by |
|---|---|
| `pyPS4Controller` | `stateMachine.py`, `jumpTest.py`, `legacy/*` — reads the DualShock 4 from `/dev/input/js0` |
| `pyserial` | `stateMachine.py`, `jumpTest.py`, `legacy/*` — talks to the Servo 2040 over `/dev/ttyACM0` |
| `RPi.GPIO` | `stateMachine.py`, `tools/Pincer.py` — PWM for the pincer servo on GPIO 17 |
| `gpiozero` | `tools/GPIOTest.py`, `legacy/Controller.py` — limit-switch inputs |

### 2.3 Permissions

Serial ports belong to the `dialout` group and joystick devices to `input`.

```bash
sudo usermod -aG dialout,input $USER
```

Log out and back in (or reboot) for the group change to apply.

### 2.4 Pair the PS4 controller

Put the controller in pairing mode by holding **Share** and the **PS** button together
until the light bar flashes rapidly. Then, on the Pi:

```bash
bluetoothctl
```

Inside the `bluetoothctl` prompt:

```
agent on
default-agent
scan on
```

Wait for a line like `[NEW] Device AA:BB:CC:DD:EE:FF Wireless Controller`, then:

```
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
exit
```

Because of `trust`, the controller reconnects on its own next time you press the PS button.
Confirm Linux sees it as a joystick:

```bash
ls -l /dev/input/js*
```

You want `/dev/input/js0`. If you have more than one controller or joystick attached it may
be `js1`; the scripts are hard-coded to `js0`. Optional: `sudo apt install joystick` then
`jstest /dev/input/js0` prints live axis and button values.

### 2.5 Connect the Servo 2040

Plug the Servo 2040 into any Pi USB port. It shows up as a USB serial device:

```bash
ls -l /dev/ttyACM*
```

You want `/dev/ttyACM0` (the scripts are hard-coded to it). To prove the link works before
involving the controller, send one packet by hand. This centres the three servos of leg L1:

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200); s.write(b'LEG:L1,S1:90,S2:90,S3:90\n')"
```

If the servos move, the board, the firmware and the cable are all fine. The packet format is
described in [PROTOCOL.md](PROTOCOL.md).

### 2.6 First run

```bash
cd RPI
source ../venv/bin/activate     # if not already active
python3 stateMachine.py
```

`stateMachine.py` does `from helperClasses.Point import Point`, which Python resolves from the
script's own folder. Run it by path (as above, or `python3 RPI/stateMachine.py` from the repo
root) and keep `RPI/helperClasses/` next to it.

On start the program:

1. opens `/dev/ttyACM0`,
2. starts a reader thread that echoes anything the board prints as `Servo2040 -> …`,
3. starts the movement thread, which moves all six legs to the idle stance, then
4. blocks in `controller.listen()`, waiting for `/dev/input/js0`.

Keep the robot lifted off the ground for the first run and press the controls in
[CONTROLS.md](CONTROLS.md) one at a time. `Ctrl+C` stops the program; the servos hold their
last position until power is cut.

## 3. Running the other scripts

| Script | Command | Needs |
|---|---|---|
| Interactive IK calculator | `python3 RPI/tools/IKtest.py` | Nothing. Works on a laptop |
| Limit switch check | `python3 RPI/tools/GPIOTest.py` | A normally-open switch between GPIO 17 and GND |
| Pincer sweep | `python3 RPI/tools/Pincer.py` | Pincer servo signal on GPIO 17 |
| Jump experiment | `cd RPI && python3 jumpTest.py` | Servo 2040 + controller; drives legs L1, L3, R1, R3 only |
| Legacy 4-leg controller | `cd RPI/legacy && python3 Controller.py` | Servo 2040 + controller + limit switches on GPIO 17/22/23/25 |

## 4. Troubleshooting

**`ModuleNotFoundError: No module named 'helperClasses'`**
`RPI/helperClasses/` is not next to the script any more (moved or copied). Put the script back
in `RPI/` and run `python3 RPI/stateMachine.py`.

**`ModuleNotFoundError: No module named 'pyPS4Controller'`** (or `serial`)
The venv is not active. `source venv/bin/activate` from the repo root.

**`serial.serialutil.SerialException: [Errno 13] Permission denied: '/dev/ttyACM0'`**
You are not in the `dialout` group yet, or have not logged out since being added.

**`[Errno 2] No such file or directory: '/dev/ttyACM0'`**
The board is unplugged, or it is in bootloader mode (a drive called `RPI-RP2` appears
instead). Unplug, make sure the BOOT button is not held, plug back in.

**`[Errno 2] No such file or directory: '/dev/input/js0'`**
The controller is not connected. Press the PS button; if nothing happens, repeat 2.4.

**`RuntimeError: Cannot determine SoC peripheral base address`** on `import RPi.GPIO`
The classic `RPi.GPIO` package does not support the Pi 5 or some newer kernels. Inside the
venv run `pip uninstall RPi.GPIO && pip install rpi-lgpio`. It is a drop-in replacement, so
the code does not change.

**`error: externally-managed-environment`** from pip
Same cause as above: pip is running against the system Python. Activate the venv.

**The board echoes `Unknown leg name:` or `Servo command with no/unknown LEG:`**
The packet did not start with a valid `LEG:` field. Valid names are `L1 L2 L3 R1 R2 R3`.

**Servos jitter, the board reboots, or the Pi's USB drops out**
Servo power is inadequate. Feed the Servo 2040's servo power input from a proper supply;
18 servos can draw far more than a USB port provides.

**Legs move but in the wrong direction / a leg is mirrored**
Check that each leg's three servos are plugged into the channels listed in
[HARDWARE.md](HARDWARE.md). The right-side legs are mirrored in software, so swapping a
left and right leg produces exactly this symptom.
