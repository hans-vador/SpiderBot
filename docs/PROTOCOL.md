# Serial protocol: Pi → Servo 2040

A tiny line-based text protocol. One line per leg update, newline-terminated, over the
Servo 2040's USB CDC serial port.

| Setting | Value | Where |
|---|---|---|
| Device on the Pi | `/dev/ttyACM0` | `SERIAL_PORT` in `RPI/stateMachine.py` |
| Baud | 115200 (8N1) | `BAUD_RATE` in `RPI/stateMachine.py` |
| Read timeout (Pi side) | 0.1 s | `serial.Serial(..., timeout=0.1)` |
| Encoding | UTF-8, `\n` line ending | `send_command()` |

## Packet format

```
LEG:<name>,S1:<angle>,S2:<angle>,S3:<angle>\n
```

| Field | Values | Meaning |
|---|---|---|
| `LEG` | `L1` `L2` `L3` `R1` `R2` `R3` | Which leg the following servo fields apply to. Must come first |
| `S1` | number | Coxa angle in degrees |
| `S2` | number | Femur angle in degrees |
| `S3` | number | Tibia angle in degrees |

Examples the Pi actually sends (angles are floats straight out of the IK, not rounded):

```
LEG:L1,S1:90.0,S2:48.317544478101354,S3:50.75958604579521
LEG:R2,S1:90.0,S2:128.71735736801597,S3:116.5494284363997
```

Every foot position goes through `body_to_leg_frame()` before the IK (±45° for the corner
legs), which is why every leg is sent `S1:90.0` at the idle stance. `tools/IKtest.py` skips
that rotation and gives `S1:135` for the same L1 point.

## Firmware behaviour (`Servo2040/micropython/main.py`)

For each received line, `apply_packet()`:

1. Splits on `,`, then each part on the first `:`. Channel names are upper-cased and
   whitespace-trimmed and values are trimmed, so `leg:L1, s1 : 90` works. Leg **names** are
   case-sensitive: `l1` is rejected.
2. Parts without a `:` are reported as `Invalid part:` and skipped.
3. `LEG:` sets the current leg. An unknown name prints `Unknown leg name:` and every servo
   field after it is ignored (`Servo command with no/unknown LEG:`).
4. `S1` / `S2` / `S3` are converted to `float`, clamped to `0 … 180`, and written to that
   leg's first / second / third servo per the channel map in [HARDWARE.md](HARDWARE.md).
   Non-numeric values print `Invalid angle (not a number):`.
5. Anything else prints `Unknown channel:`.

Fields are applied in the order they arrive, so a packet may carry one, two or all three
servo fields. The firmware prints `Received: …`, `Leg: <name>` for each `LEG:` field, and one
`Servo n -> value` line per servo written. The echoed value is the text from the packet, not
the clamped angle: `S1:200` echoes `Servo 1 -> 200` while the servo goes to 180. All of that
goes back over the same USB serial link.

## Pi-side behaviour

- `send_command()` remembers the last string it sent and **skips** a packet that is
  byte-for-byte identical to it. Consecutive identical updates for the same leg are therefore
  not re-sent, but the same angles for a different leg are.
- A reader thread calls `readline()` continuously and prints every non-empty line from the
  board as `Servo2040 -> <line>`. It is display-only; nothing is parsed.
- There is no acknowledgement or checksum. If a line is lost the next update for that leg
  simply replaces it.

## Talking to the board by hand

Useful for checking wiring without the controller:

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200); s.write(b'LEG:R3,S1:90,S2:90,S3:90\n')"
```

Or open an interactive terminal (`sudo apt install screen`, then `screen /dev/ttyACM0 115200`)
and type packets followed by Enter. Quit `screen` with `Ctrl+A` then `k`. Avoid `Ctrl+C`
in that terminal: with the MicroPython firmware it interrupts `main.py` and drops you at the
REPL. Press `Ctrl+D` there (soft reset) or power-cycle the board to restart `main.py`.

## Older format

`Servo2040/circuitpython/code.py` parses the same `LEG:` packets but has no error handling: a
servo field before any `LEG:`, or an unknown or lower-case leg name, raises an uncaught
`KeyError` and stops the program (reset the board to recover). Its boot banner still says
`Expected packet format: S2:ANGLE,S3:ANGLE`; ignore that.

`legacy/ControllerMac.py` predates the `LEG:` field and sends bare `S1:…,S2:…,S3:…` lines,
which `main.py` rejects (`Servo command with no/unknown LEG:`) and `code.py` crashes on.
