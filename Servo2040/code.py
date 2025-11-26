import time
import board
import pwmio
import usb_cdc
from adafruit_motor import servo

# ---------------------------------------
# Servo Setup
# ---------------------------------------
PWM_FREQUENCY = 50
PULSE_RANGE = (500, 2500)  # microseconds

# Servo 1
pwm1 = pwmio.PWMOut(board.SERVO_1, frequency=PWM_FREQUENCY)
servo1 = servo.Servo(pwm1, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

# Servo 2
pwm2 = pwmio.PWMOut(board.SERVO_2, frequency=PWM_FREQUENCY)
servo2 = servo.Servo(pwm2, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 3
pwm3 = pwmio.PWMOut(board.SERVO_3, frequency=PWM_FREQUENCY)
servo3 = servo.Servo(pwm3, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

# ---------------------------------------
# Serial Setup
# ---------------------------------------
serial = usb_cdc.data if usb_cdc.data else usb_cdc.console
buffer = bytearray()

print("IK Servo Bridge Ready")
print("Expected packet format: S2:ANGLE,S3:ANGLE")

servos = {
    1: servo1,
    2: servo2,
    3: servo3
}

pinOffsets = {
    "L1": 0,
    "L2": 3,
    "L3": 6,
    "R1": 9,
    "R2": 13,
    "R3": 16
}

# ---------------------------------------
# Helper: Apply angle safely
# ---------------------------------------
def set_servo(servo_obj, angle):
    try:
        angle = float(angle)
        if angle < 0: angle = 0
        if angle > 180: angle = 180
        servo_obj.angle = angle
        return True
    except Exception as e:
        print("Invalid angle:", angle, e)
        return False


# ---------------------------------------
# Parse the command string
# ---------------------------------------
def apply_packet(packet: str):
    """
    Packet example:
    S2:45,S3:90
    """

    print("Received:", packet)

    parts = packet.split(",")
    pinOffset = 0
    
    for part in parts:
        if ":" not in part:
            print("Invalid part:", part)
            continue

        channel, value = part.split(":", 1)
        channel = channel.strip().upper()
        value = value.strip()
        

        if channel == "LEG":
            pinOffset = pinOffsets[value]
            print(f"Leg:{value}, Offset:{pinOffset}")

        if channel == "S1":
            if set_servo(servos[pinOffset + 1], value):
                print(f"Servo 1 -> {value}")

        elif channel == "S2":
            if set_servo(servos[pinOffset + 2], value):
                print(f"Servo 2 -> {value}")

        elif channel == "S3":
            if set_servo(servos[pinOffset + 3], value):
                print(f"Servo 3 -> {value}")

        else:
            print("Unknown channel:", channel)


# ---------------------------------------
# Main Loop
# ---------------------------------------
while True:
    if serial.in_waiting > 0:
        data = serial.read(serial.in_waiting)
        if data:
            buffer.extend(data)

            while b"\n" in buffer:
                line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)

                try:
                    decoded = line.decode("utf-8").strip()
                except UnicodeError:
                    print("Undecodable serial input")
                    continue

                if decoded:
                    apply_packet(decoded)

    time.sleep(0.01)