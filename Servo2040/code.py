import time

import board
import pwmio
import usb_cdc
from adafruit_motor import servo


SERVO_PIN = board.GP0  # Servo output 1 on the Servo 2040
PWM_FREQUENCY = 50
PULSE_RANGE = (500, 2500)  # microseconds

FORWARD_ANGLE = 120
BACK_ANGLE = 60
CENTER_ANGLE = 90


class ServoBridge:
    def __init__(self):
        pwm = pwmio.PWMOut(SERVO_PIN, frequency=PWM_FREQUENCY)
        self._servo = servo.Servo(pwm, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])
        self._servo.angle = CENTER_ANGLE
        # Prefer the data USB channel when enabled, otherwise use the default console.
        self._serial = usb_cdc.data if usb_cdc.data else usb_cdc.console
        self._buffer = bytearray()
        print("Servo bridge ready. Send FORWARD/BACK/CENTER commands.")

    def _apply_command(self, command: str) -> None:
        cmd = command.upper()
        if cmd == "FORWARD":
            self._servo.angle = FORWARD_ANGLE
        elif cmd == "BACK":
            self._servo.angle = BACK_ANGLE
        elif cmd == "CENTER":
            self._servo.angle = CENTER_ANGLE
        else:
            print(f"Unknown command: {cmd}")
            return
        print(f"Moved servo to {cmd.lower()}.")

    def loop(self):
        while True:
            if self._serial.in_waiting > 0:
                data = self._serial.read(self._serial.in_waiting)
                if data:
                    self._buffer.extend(data)
                    while b"\n" in self._buffer:
                        line, _, remainder = self._buffer.partition(b"\n")
                        self._buffer = bytearray(remainder)
                        try:
                            decoded = line.decode("utf-8").strip()
                        except UnicodeError:
                            print("Received undecodable bytes; ignoring.")
                            continue
                        if decoded:
                            self._apply_command(decoded)
            time.sleep(0.01)


ServoBridge().loop()
