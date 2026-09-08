# Hardware reference

Physical layout of the robot as the code expects it. Each table names the source file it
matches, so if you rewire something, update both.

## Parts

| Part | Role |
|---|---|
| Raspberry Pi (with Bluetooth, 40-pin header) | Runs `RPI/stateMachine.py`; talks to the controller and the Servo 2040 |
| Pimoroni Servo 2040 | RP2040 board with 18 servo headers; runs `Servo2040/micropython/main.py` |
| 18 × hobby servos (0–180°) | Three per leg |
| Servo power supply | Into the Servo 2040's servo power input. Must handle 18 servos at once |
| Sony DualShock 4 | Bluetooth, appears on the Pi as `/dev/input/js0` |
| USB-A → USB-C cable | Pi ↔ Servo 2040 data link, appears on the Pi as `/dev/ttyACM0` |
| Pincer servo (optional) | Signal from Pi GPIO 17 (BCM numbering) |
| Limit switches (optional, legacy) | Normally-open, one per leg, to GND. Used only by `legacy/Controller.py` and `tools/GPIOTest.py` |

## Leg names and where they sit

Legs are named by side (`L` / `R`) and a number. The numbers are **not** front-to-back
symmetric between sides; `body_to_leg_frame()` in `RPI/stateMachine.py` is the source of truth:

```
              FRONT
        L3 ─────────── R1      ← hips angled 45°
        L2 ─── body ─── R2      ← hips straight out (0°)
        L1 ─────────── R3      ← hips angled -45°
              REAR
```

Each leg has three servos, always in this order:

| Packet field | Joint | Motion |
|---|---|---|
| `S1` | Coxa | Hip swing in the horizontal plane |
| `S2` | Femur | Hip lift |
| `S3` | Tibia | Knee |

Angles are in degrees, 0–180, with 90 as the mechanical centre. The firmware clamps anything
outside that range. Right-side legs are mirrored in software (`IKEngine.calculate()` flips
`S2` and `S3` for `R1`, `R2`, `R3`), so a leg built for the left side will move backwards if
plugged into a right-side channel.

## Servo 2040 channel map

Defined identically in `Servo2040/micropython/main.py` and `Servo2040/circuitpython/code.py`.
Header `SERVO_n` on the board is GPIO `n-1` on the RP2040.

| Leg | S1 (coxa) | S2 (femur) | S3 (tibia) |
|---|---|---|---|
| L1 | SERVO_1 | SERVO_2 | SERVO_3 |
| R1 | SERVO_4 | SERVO_5 | SERVO_6 |
| L3 | SERVO_7 | SERVO_8 | SERVO_9 |
| R3 | SERVO_10 | SERVO_11 | SERVO_12 |
| L2 | SERVO_13 | SERVO_14 | SERVO_15 |
| R2 | SERVO_16 | SERVO_17 | SERVO_18 |

Servo signal details:

- **MicroPython (`main.py`)** uses Pimoroni's `ServoCluster`, which drives all 18 channels
  from the RP2040's PIO. Angles 0–180 are mapped to ±90 around centre before being handed to
  `ServoCluster.value()`, which uses the library's default angular calibration.
- **CircuitPython (`code.py`)** uses `pwmio` + `adafruit_motor.servo` at 50 Hz with a
  500–2500 µs pulse range. The RP2040 only has 16 hardware PWM channels, so `pwmio` cannot
  drive all 18 headers independently, which is why this version was superseded.

## Leg geometry

From `RPI/helperClasses/IKEngine.py` (`legacy/Controller.py` carries the same three numbers;
`tools/IKtest.py` has only `FEMUR` and `TIBIA`):

| Link | Length |
|---|---|
| `COXA` | 43 |
| `FEMUR` | 60 |
| `TIBIA` | 110 |

All lengths and coordinates are in millimetres. Only `FEMUR` and `TIBIA` enter the IK maths;
`COXA` is there for reference and is not used by `calculate()`.

## Foot coordinates

Each foot is tracked as a `Point(x, y, z)`:

- `z` is vertical. Negative is below the hip. The idle stance uses `z = -70`.
- `y` points outward from the body along the leg.
- `x` runs along the body.

Before the IK runs, `body_to_leg_frame()` rotates the corner legs (`L3`/`R1` by +45°,
`L1`/`R3` by −45°) so that a straight-line stroke in the body frame maps onto the angled
hips. Leg `L2` additionally gets `+20` added to `z` inside `_update_ik()` to correct for its
mounting height.

Idle stance (`stateMachine.py`):

| Legs | Point |
|---|---|
| L3, R1 | `(-35, 35, -70)` |
| L1, R3 | `( 35, 35, -70)` |
| L2, R2 | `(  0, 70, -70)` |

## Raspberry Pi GPIO (BCM numbering)

| GPIO | Script | Purpose |
|---|---|---|
| 17 | `stateMachine.py`, `tools/Pincer.py` | Pincer servo, 50 Hz PWM via `RPi.GPIO`. Duty 1 % ≈ 0°, 12 % ≈ 180° |
| 17 | `tools/GPIOTest.py`, `legacy/Controller.py` | Limit switch for leg L1 (`gpiozero.Button`, internal pull-up, normally-open to GND) |
| 22 | `legacy/Controller.py` | Limit switch, leg L3 |
| 23 | `legacy/Controller.py` | Limit switch, leg R1 |
| 25 | `legacy/Controller.py` | Limit switch, leg R3 |

GPIO 17 does double duty: pincer PWM in the main program, limit switch in the older scripts.
Do not wire both at once.

The limit switches are disabled in the main program: `Leg` ignores the pin argument that
`stateMachine.py` passes in, and `tryLevel` stays `False`, so `gait()` never reads them. Wire
them only if you are running `legacy/Controller.py` or `tools/GPIOTest.py`.

A Pi GPIO pin can only provide the servo's *signal*. Power the pincer servo from a separate
5 V supply and tie its ground to the Pi's ground.

## Servo 2040 ↔ Pi link

- USB-C on the Servo 2040 to any USB-A port on the Pi.
- Appears as `/dev/ttyACM0`. The Pi scripts open it at 115200 baud, 8N1 (`BAUD_RATE`). The
  firmware is a plain USB CDC device and neither sets nor depends on the baud rate.
- The same link carries the board's `print()` output back to the Pi, where the reader thread
  displays it as `Servo2040 -> …`.
