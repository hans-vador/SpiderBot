from pyPS4Controller.controller import Controller
import time
import threading

#test

class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.is_moving_forward = False
        self.movement_thread = None
        self.l3_value = 0  # Add this to store L3 value

    def move_forward(self):
        while self.is_moving_forward:
            print(f"leg move forward, L3 value: {self.l3_value}")
            time.sleep(0.1)


    def start_movement(self):
        if not self.movement_thread or not self.movement_thread.is_alive():
            self.is_moving_forward = True
            self.movement_thread = threading.Thread(target=self.move_forward)
            self.movement_thread.daemon = True  # This ensures thread stops when program exits
            self.movement_thread.start()

    def stop_movement(self):
        self.is_moving_forward = False
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)
    
    def on_L3_up(self, value):
        self.l3_value = value  # Store the value
        if value < -10000:
            self.start_movement()
        else:
            self.stop_movement()

    # ...existing code with empty pass methods...

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
    def on_L3_down(self, value): pass
    def on_L3_left(self, value): pass
    def on_L3_right(self, value): pass
    def on_L3_y_at_rest(self): pass
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