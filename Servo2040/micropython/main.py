# main.py  --  Servo 2040 IK bridge in MicroPython using ServoCluster
#
# Expects lines like:
#   LEG:L1,S1:45,S2:90,S3:135
# over USB serial (same as your current setup on /dev/ttyACM0).

import gc
import time
import sys

from servo import ServoCluster, servo2040

# --------------------------------------------------------------------
# ServoCluster setup (all 18 pins)
# --------------------------------------------------------------------
gc.collect()

# SERVO_1 .. SERVO_18 are GP0..GP17 on the RP2040
START_PIN = servo2040.SERVO_1
END_PIN   = servo2040.SERVO_18

# Use PIO 0, State Machine 0 for the cluster
servos = ServoCluster(pio=0, sm=0, pins=list(range(START_PIN, END_PIN + 1)))

# Turn everything on
servos.enable_all()
time.sleep(0.5)


# --------------------------------------------------------------------
# Small wrapper so each "servo" looks like the CircuitPython one
# with a .angle property in [0, 180].
#
# ServoCluster.value(index, value) expects degrees offset from centre,
# where 0 = centre, +ve and -ve swing either side.
# We'll treat input as 0–180° and map 90° -> 0 offset.
# --------------------------------------------------------------------
class ClusterServo:
    def __init__(self, cluster, index, centre=90.0):
        self._cluster = cluster
        self._index = index
        self._centre = centre
        self._angle = centre  # start at centre
        self._apply()

    def _apply(self):
        offset = self._angle - self._centre  # map 0–180 to ± around centre
        # Apply immediately
        self._cluster.value(self._index, offset, load=True)

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, deg):
        # Coerce and clamp
        try:
            deg = float(deg)
        except Exception:
            return
        if deg < 0:
            deg = 0
        elif deg > 180:
            deg = 180
        self._angle = deg
        self._apply()


# --------------------------------------------------------------------
# Build 18 servo objects (indices 0..17 -> SERVO_1..SERVO_18)
# --------------------------------------------------------------------
servo1  = ClusterServo(servos, 0)
servo2  = ClusterServo(servos, 1)
servo3  = ClusterServo(servos, 2)
servo4  = ClusterServo(servos, 3)
servo5  = ClusterServo(servos, 4)
servo6  = ClusterServo(servos, 5)
servo7  = ClusterServo(servos, 6)
servo8  = ClusterServo(servos, 7)
servo9  = ClusterServo(servos, 8)
servo10 = ClusterServo(servos, 9)
servo11 = ClusterServo(servos, 10)
servo12 = ClusterServo(servos, 11)
servo13 = ClusterServo(servos, 12)
servo14 = ClusterServo(servos, 13)
servo15 = ClusterServo(servos, 14)
servo16 = ClusterServo(servos, 15)
servo17 = ClusterServo(servos, 16)
servo18 = ClusterServo(servos, 17)

# Leg mapping identical to your CircuitPython version
legServos = {
    "L1": [servo1,  servo2,  servo3],
    "R1": [servo4,  servo5,  servo6],
    "L3": [servo7,  servo8,  servo9],
    "R3": [servo10, servo11, servo12],
    "L2": [servo13, servo14, servo15],
    "R2": [servo16, servo17, servo18],
}


# --------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------
def set_servo(servo_obj, angle):
    """Clamp and set angle on a ClusterServo."""
    try:
        angle = float(angle)
    except Exception as e:
        print("Invalid angle (not a number):", angle, e)
        return False

    if angle < 0:
        angle = 0
    if angle > 180:
        angle = 180

    try:
        servo_obj.angle = angle
        return True
    except Exception as e:
        print("Error setting servo:", e)
        return False


def apply_packet(packet: str):
    """
    Parse and apply a command like:
        LEG:L1,S1:45,S2:90,S3:135
    """
    print("Received:", packet)

    parts = packet.split(",")
    leg = None

    for part in parts:
        if ":" not in part:
            if part.strip():
                print("Invalid part:", part)
            continue

        channel, value = part.split(":", 1)
        channel = channel.strip().upper()
        value = value.strip()

        if channel == "LEG":
            leg = value
            print("Leg:", leg)
            if leg not in legServos:
                print("Unknown leg name:", leg)
            continue

        # Ignore servo commands if leg hasn't been set / unknown
        if not leg or leg not in legServos:
            print("Servo command with no/unknown LEG:", channel, value)
            continue

        if channel == "S1":
            if set_servo(legServos[leg][0], value):
                print("Servo 1 ->", value)

        elif channel == "S2":
            if set_servo(legServos[leg][1], value):
                print("Servo 2 ->", value)

        elif channel == "S3":
            if set_servo(legServos[leg][2], value):
                print("Servo 3 ->", value)

        else:
            print("Unknown channel:", channel)


# --------------------------------------------------------------------
# Main loop: read lines from USB CDC (sys.stdin) and apply
# --------------------------------------------------------------------
print("IK Servo Bridge Ready (MicroPython + ServoCluster)")
print("Expected packet format: LEG:L1,S1:ANGLE,S2:ANGLE,S3:ANGLE")

while True:
    try:
        # Blocking readline from USB serial
        line = sys.stdin.readline()
    except Exception as e:
        print("stdin read error:", e)
        time.sleep(0.05)
        continue

    if not line:
        # No data; tiny sleep to avoid busy-wait
        time.sleep(0.01)
        continue

    # On MicroPython this is usually already a str, but just in case:
    if isinstance(line, bytes):
        try:
            decoded = line.decode("utf-8").strip()
        except UnicodeError:
            print("Undecodable input:", line)
            continue
    else:
        decoded = line.strip()

    if decoded:
        apply_packet(decoded)
