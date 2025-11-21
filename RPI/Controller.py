from pyPS4Controller.controller import Controller
import serial

SERIAL_PORT = "/dev/ttyACM0"  # Adjust if your Servo 2040 enumerates differently.
BAUD_RATE = 115200

class MyController(Controller):
    def __init__(self, serial_port=SERIAL_PORT, **kwargs):
        super().__init__(**kwargs)
        self.serial_port = serial_port
        self.serial_conn = serial.Serial(serial_port, BAUD_RATE, timeout=0)
        self.current_command = None
        self.send_command("CENTER")

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

    def on_L3_up(self, value):
        if value < -10000:
            self.send_command("FORWARD")

    def on_L3_down(self, value):
        if value > 10000:
            self.send_command("BACK")

    def on_L3_y_at_rest(self):
        self.send_command("CENTER")

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
