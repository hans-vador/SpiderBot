from pyPS4Controller.controller import Controller
import serial
import threading
import time
import math

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
        A = math.degrees(math.atan2(-z, y))

        J2 = B - A

        #J1
        J1 = math.degrees(math.atan(x/y))
        J1 = 90 + J1

        self.S1Angle = J1
        self.S2Angle = 90 - J2
        self.S3Angle = J3

        return self.S1Angle, self.S2Angle, self.S3Angle

class MyController(Controller):
    def __init__(self, serial_port=SERIAL_PORT, **kwargs):
        self.x = 0
        self.y = 120
        self.z = 0
        self.ik = IKEngine()
        super().__init__(**kwargs)

        #Initial State
        self.state = "idle"
        self.xMultiplier = 0
        self.yMultiplier = 0
        self.zMultiplier = 0
        self.speed = 0.2
        self.moved = False
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
    
    def _update_ik(self):
        S1, S2, S3 = self.ik.calculate(self.x, self.y, self.z)
        self.desired_command = f"S1:{S1},S2:{S2},S3:{S3}"

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
            print(f"X Axis: {self.x}")
            print(f"Y Axis: {self.y}")
            print(f"Z Axis: {self.z}")
        except serial.SerialException as exc:
            print(f"Serial write failed: {exc}")

    # -------------------------------
    # BACKGROUND LOOP (SENDS IK PACKETS)
    # -------------------------------
    def _movement_loop(self):
        while True:
            with self._command_lock:
                target = self.desired_command  # whatever YOU compute
            
            if self.state == "idle":
                 if target:
                      self.send_command(target)
            if self.xMultiplier != 0:
                 self.x += self.xMultiplier * self.speed
                 self.moved = True
            if self.yMultiplier != 0:
                 self.y += self.yMultiplier * self.speed
                 self.moved = True
            if self.zMultiplier != 0:
                 self.z += self.zMultiplier * self.speed
                 self.moved = True
            if self.moved:
                 self._update_ik()
                 self.send_command(target)
                 self.moved = False
                 
            
            time.sleep(0.02)  # 50 Hz update for smooth robotics

    # -------------------------------
    # SET TARGET COMMAND (IK STRING)
    # -------------------------------
    def _set_target_command(self, command: str):
        with self._command_lock:
            self.desired_command = command

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
                print(f"Servo2040 -> {decoded}")
    

    def on_up_arrow_press(self):
            with self._command_lock:
                self.z += 30
                self._update_ik()

    def on_down_arrow_press(self):
            with self._command_lock:
                self.z -= 30
                self._update_ik()

    def on_left_arrow_press(self):
            with self._command_lock:
                self.x -= 5
                self._update_ik()

    def on_right_arrow_press(self):
            with self._command_lock:
                self.x += 5
                self._update_ik()

    # Empty methods for all other controls to silence them
    def on_x_press(self): 
         self.speed += 0.2
    def on_x_release(self): pass
    def on_triangle_press(self): 
         self.y = 45
         self._update_ik()
    def on_triangle_release(self): pass
    def on_circle_press(self): 
         self.y += 5
         self._update_ik()
    def on_circle_release(self): pass
    def on_square_press(self): 
         self.y -= 5
         self._update_ik()
    def on_square_release(self): pass
    def on_L1_press(self): pass
    def on_L1_release(self): pass
    def on_L2_press(self, value): pass
    def on_L2_release(self): pass
    def on_R1_press(self): pass
    def on_R1_release(self): pass
    def on_R2_press(self, value): pass
    def on_R2_release(self): pass
    def on_up_down_arrow_release(self): pass
    def on_left_right_arrow_release(self): pass
    def on_L3_x_at_rest(self): pass
    def on_L3_y_at_rest(self): pass
    def on_L3_press(self): pass
    def on_L3_release(self): pass
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