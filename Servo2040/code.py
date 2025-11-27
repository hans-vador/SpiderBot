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

# Servo 1
pwm4 = pwmio.PWMOut(board.SERVO_4, frequency=PWM_FREQUENCY)
servo4 = servo.Servo(pwm4, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

# Servo 2
pwm5 = pwmio.PWMOut(board.SERVO_5, frequency=PWM_FREQUENCY)
servo5 = servo.Servo(pwm5, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 6
pwm6 = pwmio.PWMOut(board.SERVO_6, frequency=PWM_FREQUENCY)
servo6 = servo.Servo(pwm6, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 7
pwm7 = pwmio.PWMOut(board.SERVO_7, frequency=PWM_FREQUENCY)
servo7 = servo.Servo(pwm7, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 8
pwm8 = pwmio.PWMOut(board.SERVO_8, frequency=PWM_FREQUENCY)
servo8 = servo.Servo(pwm8, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 9
pwm9 = pwmio.PWMOut(board.SERVO_9, frequency=PWM_FREQUENCY)
servo9 = servo.Servo(pwm9, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 10
pwm10 = pwmio.PWMOut(board.SERVO_10, frequency=PWM_FREQUENCY)
servo10 = servo.Servo(pwm10, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 11
pwm11 = pwmio.PWMOut(board.SERVO_11, frequency=PWM_FREQUENCY)
servo11 = servo.Servo(pwm11, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

#Servo 12
pwm12 = pwmio.PWMOut(board.SERVO_12, frequency=PWM_FREQUENCY)
servo12 = servo.Servo(pwm12, min_pulse=PULSE_RANGE[0], max_pulse=PULSE_RANGE[1])

# ---------------------------------------
# Serial Setup
# ---------------------------------------
serial = usb_cdc.data if usb_cdc.data else usb_cdc.console
buffer = bytearray()

print("IK Servo Bridge Ready")
print("Expected packet format: S2:ANGLE,S3:ANGLE")


legServos = {
    "L1": {servo1, servo2, servo3},
    "R1": {servo4, servo5, servo6},
    "L3": {servo7, servo8, servo9},
    "R3": {servo10, servo11, servo12}
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
    leg = "none"
    
    for part in parts:
        if ":" not in part:
            print("Invalid part:", part)
            continue

        channel, value = part.split(":", 1)
        channel = channel.strip().upper()
        value = value.strip()
        

        if channel == "LEG":
            leg = value
            print(f"Leg:{value}")
            continue

        if channel == "S1":
            if set_servo(legServos[leg][0], value):
                print(f"Servo 1 -> {value}")

        elif channel == "S2":
            if set_servo(legServos[leg][1], value):
                print(f"Servo 2 -> {value}")

        elif channel == "S3":
            if set_servo(legServos[leg][2], value):
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