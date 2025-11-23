from pyPS4Controller.controller import Controller
import serial
import threading
import time
import math

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
COXA = 43
FEMUR = 60
TIBIA = 104

class IKEngine:
    def __init__(self):
        self.S2Angle = 0
        self.S3Angle = 0

    def calculate(self, y, z):
        L = math.sqrt(y**2+z**2)

        J3 = math.acos((FEMUR**2 + TIBIA**2 - L**2)/(2*FEMUR*TIBIA))
        J3 = math.degrees(J3)

        B = math.acos((L**2 + FEMUR**2 - TIBIA**2)/(2*L*FEMUR))
        B = math.degrees(B)

        A = math.atan2(z, y)
        A = math.degrees(A)

        J2 = B - A

        self.S2Angle = 90 - J2
        self.S3Angle = J3

        return self.S2Angle, self.S3Angle 
        


class MyController(Controller):
    # RME: Requirements - serial_port (optional, defaults to SERIAL_PORT), **kwargs for parent Controller
    #      Modifies - Initializes self.serial_port, self.serial_conn, self.current_command, 
    #                 self.desired_command, self._command_lock, and starts reader_thread and movement_thread
    #      Effects - Creates serial connection to Servo 2040, initializes controller state, 
    #                and starts background threads for serial reading and movement control
    def __init__(self, serial_port=SERIAL_PORT, **kwargs):
        self.y = 0
        self.z = 0
        self.ik = IKEngine()
        super().__init__(**kwargs)

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
        S2, S3 = self.ik.calculate(self.y, self.z)
        self.desired_command = f"S2:{S2},S3:{S3}"

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
    

    def on_L3_up(self, value):
        if value < -10000:
            with self._command_lock:
                self.z += 1
                self._update_ik()

    def on_L3_down(self, value):
        if value > 10000:
            with self._command_lock:
                self.z -= 1
                self._update_ik()

    def on_L3_left(self, value):
        if value < -10000:
            with self._command_lock:
                self.y -= 1
                self._update_ik()

    def on_L3_right(self, value):
        if value > 10000:
            with self._command_lock:
                self.y += 1
                self._update_ik()

    # Empty methods for all other controls to silence them
    def on_x_press(self): pass
    def on_x_release(self): pass
    def on_triangle_press(self): pass
    def on_triangle_release(self): pass
    def on_circle_press(self): pass
    def on_circle_release(self): pass
    def on_square_press(self): pass
    def on_square_release(self): pass
    def on_L1_press(self): pass
    def on_L1_release(self): pass
    def on_L2_press(self, value): pass
    def on_L2_release(self): pass
    def on_R1_press(self): pass
    def on_R1_release(self): pass
    def on_R2_press(self, value): pass
    def on_R2_release(self): pass
    def on_up_arrow_press(self): pass
    def on_up_down_arrow_release(self): pass
    def on_down_arrow_press(self): pass
    def on_left_arrow_press(self): pass
    def on_left_right_arrow_release(self): pass
    def on_right_arrow_press(self): pass
    def on_L3_x_at_rest(self): pass
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