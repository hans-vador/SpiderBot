# Servo2040/ — firmware for the Pimoroni Servo 2040

The Servo 2040 is an RP2040 board with 18 servo headers. It does one job here: read
`LEG:…,S1:…,S2:…,S3:…` lines from USB serial and move servos (see
[../docs/PROTOCOL.md](../docs/PROTOCOL.md)). Two implementations exist. **Flash one, not both.**

| Folder | Runtime | Status |
|---|---|---|
| `micropython/main.py` | Pimoroni MicroPython, `ServoCluster` | **Current.** Drives all 18 servos via PIO |
| `circuitpython/code.py` | CircuitPython, `pwmio` + `adafruit_motor` | Older. Kept for reference |

Both use the same leg → channel map (see [../docs/HARDWARE.md](../docs/HARDWARE.md)).

## Option A (recommended): MicroPython

1. **Get the firmware.** Download Pimoroni's MicroPython `.uf2` from the releases page of
   [pimoroni/pimoroni-pico](https://github.com/pimoroni/pimoroni-pico/releases). There is no
   Servo 2040-specific file; use the RP2040 build named like
   `pico-v1.29.0-2-pimoroni-micropython.uf2` (not the `pico2`/`picow`/`tiny` ones). It must be
   Pimoroni's build, not stock MicroPython: `main.py` needs the bundled `servo` module.
2. **Enter the bootloader.** Hold the **BOOT** button on the Servo 2040 while plugging it
   into a computer. A USB drive named `RPI-RP2` appears.
3. **Flash.** Drag the `.uf2` onto `RPI-RP2`. The drive disappears and the board reboots
   into MicroPython.
4. **Copy the program** to the board's root as `main.py` so it runs at boot. Either open
   `micropython/main.py` in [Thonny](https://thonny.org) and use *File → Save as… → Raspberry
   Pi Pico*, or from a terminal at the repository root:

   ```bash
   pip install mpremote
   mpremote cp Servo2040/micropython/main.py :main.py
   mpremote reset
   ```

5. **Check.** Open the serial port (`screen /dev/ttyACM0 115200`, or Thonny's shell). You
   should see:

   ```
   IK Servo Bridge Ready (MicroPython + ServoCluster)
   Expected packet format: LEG:L1,S1:ANGLE,S2:ANGLE,S3:ANGLE
   ```

   Type `LEG:L1,S1:90,S2:90,S3:90` and press Enter; leg L1's three servos centre.

On boot `main.py` enables all 18 channels and sets each to 90° (centre), so expect every
servo to move once when the board powers up.

## Option B: CircuitPython

1. Download the Servo 2040 `.uf2` from
   [circuitpython.org/board/pimoroni_servo2040](https://circuitpython.org/board/pimoroni_servo2040/).
2. Hold **BOOT**, plug in, drag the `.uf2` onto `RPI-RP2`. The board reboots and reappears as a
   drive named `CIRCUITPY`.
3. Download the [CircuitPython Library Bundle](https://circuitpython.org/libraries) matching
   your CircuitPython major version and copy its `adafruit_motor` folder into `CIRCUITPY/lib/`.
4. Copy `circuitpython/code.py` to the root of `CIRCUITPY` as `code.py`. CircuitPython runs it
   automatically.

No `boot.py` is needed. `code.py` uses the USB data channel if one is enabled and otherwise
falls back to the console channel, which is what the Pi sees as `/dev/ttyACM0`. Note the
RP2040 has only 16 hardware PWM channels, so this version cannot drive all 18 headers
independently, which is why the MicroPython version replaced it.

## Getting back to the bootloader

Hold **BOOT** while plugging the board in (or while pressing its reset, if fitted). The
`RPI-RP2` drive comes back and you can flash a different `.uf2`.
