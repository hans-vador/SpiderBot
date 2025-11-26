from pyPS4Controller.controller import Controller
import serial
import threading
import time
import math
from gpiozero import Button

#GPIO TESTING


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
COXA = 43
FEMUR = 60
TIBIA = 110
SPEED = 0.2

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class IKEngine:
    def __init__(self):
        self.S1Angle = 0
        self.S2Angle = 0
        self.S3Angle = 0

    def calculate(self, x, y, z):
        L = math.sqrt((x**2 + y**2) + z**2)

        # J3
        J3 = (FEMUR**2 + TIBIA**2 - L**2) / (2 * FEMUR * TIBIA)
        J3 = math.degrees(math.acos(J3))

        # B
        B = (L**2 + FEMUR**2 - TIBIA**2) / (2 * L * FEMUR)
        B = math.degrees(math.acos(B))

        # A
        A = math.degrees(math.atan2(-z, math.sqrt(x**2+y**2)))

        J2 = B - A

        #J1
        J1 = math.degrees(math.atan(x/y))

        self.S1Angle = 90 + J1 
        self.S2Angle = 90 - J2
        self.S3Angle = J3

        return self.S1Angle, self.S2Angle, self.S3Angle


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def distance_to(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def midpoint(self, other):
        mx = (self.x + other.x) / 2.0
        my = (self.y + other.y) / 2.0
        mz = (self.z + other.z) / 2.0
        return Point(mx, my, mz)

    # -------------------------------------------
    # Arithmetic operations
    # -------------------------------------------

    # Subtraction: p3 = p1 - p2
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    # Addition: p3 = p1 + p2
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    # Scalar division: p2 = p1 / num
    def __truediv__(self, num):
        return Point(self.x / num, self.y / num, self.z / num)

    # Pretty print
    def __repr__(self):
        return f"Point({self.x}, {self.y}, {self.z})"

    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return (
            self.x == other.x and
            self.y == other.y and
            self.z == other.z
        )


class Leg:
    def __init__(self, name: str, point: Point, GPIO: int):
        self.name = name
        self.position = point
        self.target = self.position
        self.gaitCurrent = self.position
        self.gaiting = False
        self.limitSwitch = Button(GPIO, pull_up=True)
        self.leveledZ = -60
        self.tryLevel = False

    def __repr__(self):
        return f"Leg(name='{self.name}', position={self.position})"

        
class MyController(Controller):
    def __init__(self, serial_port=SERIAL_PORT, **kwargs):
        self.ik = IKEngine()
        super().__init__(**kwargs)

        #Initial State
        self.xMultiplier = 0
        self.yMultiplier = 0
        self.zMultiplier = 0
        self.speed = 0.2
        self.moved = False
        self.state = "idle"
        self.gaiting = False

        self.L1 = Leg("L1", Point(0, 120, 0), 17)

        # Serial connection to Servo2040
        self.serial_port = serial_port
        self.serial_conn = serial.Serial(serial_port, BAUD_RATE, timeout=0.1)

        # Stores whatever command string YOU want to send (IK output later)
        self.current_command = None
        self.desired_command = ""  # start empty

        # Lock for thread-safe access
        self._command_lock = threading.Lock()

        # Background threads
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()

        self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
        self.movement_thread.start()

    # -------------------------------
    # Updating IKEngine Class
    # -------------------------------
    def _update_ik(self, leg):
        S1, S2, S3 = self.ik.calculate(leg.position.x, leg.position.y, leg.position.z)
        self.desired_command = f"LEG:{leg.name},S1:{S1},S2:{S2},S3:{S3}"

    # -------------------------------
    # SERIAL SEND
    # -------------------------------
    def send_command(self, command: str):
        if command == self.current_command:
            return
        message = f"{command}\n".encode("utf-8")
        try:
            self.serial_conn.write(message)
            self.serial_conn.flush()
            self.current_command = command
            print(f"Sent command: {command}")
            print(f"X Axis: {self.L1.position.x}")
            print(f"Y Axis: {self.L1.position.y}")
            print(f"Z Axis: {self.L1.position.z}")
        except serial.SerialException as exc:
            print(f"Serial write failed: {exc}")

    # -------------------------------
    # BACKGROUND LOOP (SENDS IK PACKETS)
    # -------------------------------
    def _movement_loop(self):
        while True:
            with self._command_lock:
                target = self.desired_command  # whatever YOU compute
            
            if target:
                self.send_command(target)
            if self.xMultiplier != 0:
                 self.L1.position.x += self.xMultiplier * self.speed 
                 self.moved = True
            if self.yMultiplier != 0:
                 self.L1.position.y += self.yMultiplier * self.speed
                 self.moved = True
            if self.zMultiplier != 0:
                 self.L1.position.z += self.zMultiplier * self.speed
                 self.moved = True
            if self.moved:
                 self._update_ik(self.L1)
                 self.moved = False
            
            if self.gaiting == True:
                if self.L1.target == Point(45, 75, self.L1.leveledZ):
                    if self.L1.limitSwitch.is_pressed:
                        self.L1.leveledZ = self.L1.position.z
                        self.L1.gaitCurrent = Point(*self.L1.position.__dict__.values())
                        self.L1.target = Point(-35, 75, self.L1.leveledZ)
        
                    time.sleep(0.02)
                self.gait(self.L1, 25, Point(45, 75, -60), Point(-35, 75, self.L1.leveledZ))
                self._update_ik(self.L1)
            time.sleep(0.02)  # 50 Hz update for smooth robotics

    def moveLeg(self, leg, xOffset, yOffet, zOffset):
        self.L1.position.x += xOffset
        self.L1.position.y += yOffet
        self.L1.position.z += zOffset
        self._update_ik(leg)
    
    def gait(self, leg, speed, gaitEnd, gaitStart):

        # START gait
        if not leg.gaiting:
            leg.target = gaitEnd
            leg.gaitCurrent = Point(leg.position.x, leg.position.y, leg.position.z)
            leg.midpoint = gaitEnd.midpoint(gaitStart)
            leg.midpoint.z = leg.leveledZ + 50
            leg.gaiting = True

        threshold = 3

        if leg.target == gaitStart:
            speed = speed*2

        # Compute increment
        increment = leg.target - leg.gaitCurrent
        increment = increment / speed

        # Update POSITION IN PLACE
        if leg.tryLevel:
            leg.position.z -= 1
            if leg.limitSwitch.is_pressed:
                leg.tryLevel = False
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = gaitStart
        else:
            leg.position.x += increment.x
            leg.position.y += increment.y
            leg.position.z += increment.z

        # Check cycle transitions
        if leg.position.distance_to(leg.target) < threshold:

            if leg.target == gaitEnd:
                if leg.limitSwitch.is_pressed:
                    leg.tryLevel = False
                    leg.gaitCurrent = Point(*leg.position.__dict__.values())
                    leg.target = gaitStart
                else:
                    leg.tryLevel = True
                

            elif leg.target == gaitStart:
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = leg.midpoint
                

            elif leg.target == leg.midpoint:
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = gaitEnd
                


    # -------------------------------
    # SERIAL READER (OPTIONAL FEEDBACK)
    # -------------------------------
    def _serial_reader(self):
        while True:
            try:
                line = self.serial_conn.readline()
            except serial.SerialException as exc:
                print(f"Serial read error: {exc}")
                break

            if not line:
                continue

            decoded = line.decode("utf-8", errors="replace").strip()
            if decoded:
                #print(f"Servo2040 -> {decoded}")
                x = 0
            
    
    # -------------------------------
    # X,Y,Z Incrementing Functions:
    # -------------------------------
    def on_up_arrow_press(self):
        with self._command_lock:
            self.L1.position.z += 5
            self._update_ik(self.L1)

    def on_down_arrow_press(self):
        with self._command_lock:
            self.L1.position.z -= 5
            self._update_ik(self.L1)

    def on_left_arrow_press(self):
        with self._command_lock:
            self.L1.position.y +=5 
            self._update_ik(self.L1)

    def on_right_arrow_press(self):
        with self._command_lock:
            self.L1.position.y -= 5
            self._update_ik(self.L1)

    def on_circle_press(self): 
        self.L1.position.x += 5
        self._update_ik(self.L1)
        
    def on_square_press(self): 
        self.L1.position.x -= 5
        self._update_ik(self.L1)
    
    # ----------------------------------------
    # Variable Speed Joystick Control:
    # ----------------------------------------
    def on_L3_up(self, value): 
        speed = value/-10000
        if speed > 1:
            self.zMultiplier = speed
        else:
            self.zMultiplier = 0

    def on_L3_down(self, value): 
        speed = value/-10000
        if speed * -1 > 1:
            self.zMultiplier = speed
        else:
            self.zMultiplier = 0

    def on_L3_left(self, value): 
        speed = value/10000
        if speed * -1 > 1:
            self.xMultiplier = speed
        else:
            self.xMultiplier = 0

    def on_L3_right(self, value): 
        speed = value/10000
        if speed > 1:
            self.xMultiplier = speed
        else:
            self.xMultiplier = 0

    # ----------------------------------------
    # Speed Manipulation For Joystick Control:
    # ----------------------------------------
    def on_triangle_press(self):
        self.speed += 0.2
    def on_x_press(self): 
        self.speed -= 0.2

    # ----------------------------------------
    # Empty methods For Other Controls
    # ----------------------------------------      
    def on_x_release(self): pass
    def on_triangle_press(self): pass
    def on_triangle_release(self): pass
    def on_circle_release(self): pass
    def on_square_release(self): pass
    def on_L1_press(self): pass
    def on_L1_release(self): pass
    def on_L2_press(self, value): pass
    def on_L2_release(self): 
        self.L1.position.x -= 5
        self._update_ik(self.L1)
    def on_R1_press(self): 
        self.gaiting = True
    def on_R1_release(self): 
        self.gaiting = False
        self.L1.gaiting = False
        self.L1.target = self.L1.position
        self.L1.gaitCurrent = self.L1.position
    def on_R2_press(self, value): pass
    def on_R2_release(self): 
        self.L1.position.x += 5
        self._update_ik(self.L1)
    def on_up_down_arrow_release(self): pass
    def on_left_right_arrow_release(self): pass
    def on_L3_x_at_rest(self): pass
    def on_L3_y_at_rest(self): pass
    def on_L3_press(self): pass
    def on_L3_release(self): pass
    def on_R3_up(self, value): pass
    def on_R3_down(self, value): pass
    def on_R3_left(self, value): pass
    def on_R3_right(self, value): pass
    def on_R3_y_at_rest(self): pass
    def on_R3_x_at_rest(self): pass
    def on_R3_press(self): pass
    def on_R3_release(self): pass
    def on_options_press(self): pass
    def on_options_release(self): pass
    def on_share_press(self): pass
    def on_share_release(self): pass
    def on_playstation_button_press(self): pass
    def on_playstation_button_release(self): pass

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()