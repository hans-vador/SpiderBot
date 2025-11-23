from pyPS4Controller.controller import Controller
import serial
import threading
import time

SERIAL_PORT = "/dev/ttyACM0"  # Adjust if your Servo 2040 enumerates differently.
BAUD_RATE = 115200

class MyController(Controller):
    # RME: Requirements - serial_port (optional, defaults to SERIAL_PORT), **kwargs for parent Controller
    #      Modifies - Initializes self.serial_port, self.serial_conn, self.current_command, 
    #                 self.desired_command, self._command_lock, and starts reader_thread and movement_thread
    #      Effects - Creates serial connection to Servo 2040, initializes controller state, 
    #                and starts background threads for serial reading and movement control
    def __init__(self, serial_port=SERIAL_PORT, **kwargs):
        super().__init__(**kwargs)
        self.serial_port = serial_port
        self.serial_conn = serial.Serial(serial_port, BAUD_RATE, timeout=0.1)
        self.current_command = None
        self.desired_command = "CENTER"
        self._command_lock = threading.Lock()
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()
        self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
        self.movement_thread.start()

    # RME: Requirements - command (str) must be a valid command string, self.serial_conn must be open
    #      Modifies - self.current_command if command differs from current
    #      Effects - Sends command to Servo 2040 via serial if command changed, prints status or error
    def send_command(self, command: str):
        """
        Send a command string to the Servo 2040 if it changed.
        """
        if command == self.current_command:
            return
        message = f"{command}\n".encode("utf-8")
        try:
            self.serial_conn.write(message)
            self.serial_conn.flush()
            self.current_command = command
            print(f"Sent command to servo: {command}")
        except serial.SerialException as exc:
            print(f"Failed to talk to Servo 2040: {exc}")

    # RME: Requirements - self._command_lock and self.desired_command must exist, self.send_command must be callable
    #      Modifies - Reads self.desired_command (with lock protection)
    #      Effects - Continuously reads desired command and sends it to servo every 0.05 seconds (runs in background thread)
    def _movement_loop(self):
        while True:
            with self._command_lock:
                target = self.desired_command
            self.send_command(target)
            time.sleep(0.05)

    # RME: Requirements - command (str) must be provided, self._command_lock must exist
    #      Modifies - self.desired_command (thread-safe update using lock)
    #      Effects - Updates the target command that will be sent to the servo in the movement loop
    def _set_target_command(self, command: str):
        with self._command_lock:
            self.desired_command = command

    # RME: Requirements - self.serial_conn must be open and connected to Servo 2040
    #      Modifies - Reads data from serial buffer
    #      Effects - Continuously reads serial messages from Servo 2040, decodes and prints them (runs in background thread)
    def _serial_reader(self):
        while True:
            try:
                line = self.serial_conn.readline()
            except serial.SerialException as exc:
                print(f"Servo 2040 serial read error: {exc}")
                break
            if not line:
                continue
            decoded = line.decode("utf-8", errors="replace").strip()
            if decoded:
                print(f"Servo2040 -> {decoded}")

    # RME: Requirements - value (int) from PS4 controller L3 stick Y-axis, self._set_target_command must be callable
    #      Modifies - self.desired_command via _set_target_command
    #      Effects - Sets command to "FORWARD" if stick pushed up significantly (value < -10000), otherwise "CENTER"
    def on_L3_up(self, value):
        if value < -10000:
            self._set_target_command("FORWARD")
        else: 
            self._set_target_command("CENTER")


    # RME: Requirements - value (int) from PS4 controller L3 stick Y-axis, self._set_target_command must be callable
    #      Modifies - self.desired_command via _set_target_command
    #      Effects - Sets command to "BACK" if stick pushed down significantly (value > 10000), otherwise "CENTER"
    def on_L3_down(self, value):
        if value > 10000:
            self._set_target_command("BACK")
        else: 
            self._set_target_command("CENTER")

    # RME: Requirements - PS4 controller L3 stick Y-axis at rest position, self._set_target_command must be callable
    #      Modifies - self.desired_command via _set_target_command
    #      Effects - Sets command to "CENTER" when L3 stick Y-axis returns to rest position
    def on_L3_y_at_rest(self):
        self._set_target_command("CENTER")

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
    def on_L3_left(self, value): pass
    def on_L3_right(self, value): pass
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
