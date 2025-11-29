from pyPS4Controller.controller import Controller
import serial
import threading
import time
import math

from helperClasses.Point import Point
from helperClasses.IKEngine import IKEngine
from helperClasses.Leg import Leg

#GPIO TESTING


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
SPEED = 0.2



        
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

        self.idlePoint = Point(0, 45, -70)
        self.L1 = Leg("L1", self.idlePoint, 17)
        self.L3 = Leg("L3", self.idlePoint, 22)
        self.R1 = Leg("R1", self.idlePoint, 23)
        self.R3 = Leg("R3", self.idlePoint, 25)

        self.legs = [self.L1, self.R1, self.R3, self.L3]
        self.currentLeg = 0

        self.changedState = True

        #controller inputs
        self.L3Vertical = 0
        self.L3Horizontal = 0
        self.R3Vertical = 0
        self.R3Horizontal = 0

        self.arrow = 0

        self.triangle = 1

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
        x_leg, y_leg, z_leg = self.body_to_leg_frame(leg)
        S1, S2, S3 = self.ik.calculate(leg.name, x_leg, y_leg, z_leg)
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
            
            if self.state == "idle":
                if self.changedState:
                    for leg in self.legs:
                        #leg.position = Point(self.idlePoint.x, self.idlePoint.y, self.idlePoint.z)
                        self._update_ik(leg)
                        self.send_command(self.desired_command)
                    self.changedState = False

                if self.L3Vertical != 0 or self.L3Horizontal != 0:
                    self.state = "walk" 
                    self.changedState = True
                elif self.triangle == -1:
                    self.state = "controlLeg"
                    self.changedState = True
                elif self.R3Vertical != 0 or self.R3Horizontal != 0:
                    self.state = "flex"
                    self.changedState = True
                
            elif self.state == "flex":
                self.L1.position.z += (self.R3Vertical - self.R3Horizontal) * self.speed
                self._update_ik(self.L1)
                self.send_command(self.desired_command)

                self.R1.position.z += (self.R3Vertical + self.R3Horizontal) * self.speed
                self._update_ik(self.R1)
                self.send_command(self.desired_command)

                self.L3.position.z -= (self.R3Vertical + self.R3Horizontal) * self.speed
                self._update_ik(self.L3)
                self.send_command(self.desired_command)

                self.R3.position.z -= (self.R3Vertical - self.R3Horizontal) * self.speed
                self._update_ik(self.R3)
                self.send_command(self.desired_command)

                if self.R3Vertical == 0 and self.R3Horizontal == 0:
                    self.state = "idle"
                    self.changedState = True

            elif self.state == "walk":

                gaitEnd = Point(45, 75, -60)
                gaitStart = Point(-45, 75, -60)
                if self.changedState:
                    for leg in self.legs:
                        #leg.position = Point(self.idlePoint.x, self.idlePoint.y, self.idlePoint.z)
                        #self._update_ik(leg)
                        #self.send_command(self.desired_command)
                        x=0
                    self.changedState = False
                for leg in self.legs:
                    with self._command_lock:
                        if leg.name == "L1" or leg.name == "L3":
                            self.gait(leg, 25, gaitEnd, gaitStart, "upperMid")
                            self._update_ik(leg)
                            self.send_command(self.desired_command)
                        elif leg.name == "R1" or leg.name == "R3":
                            self.gait(leg, 25, gaitStart, gaitEnd, "upperMid")
                            self._update_ik(leg)
                            self.send_command(self.desired_command)



                if self.L3Vertical == 0 and self.L3Horizontal == 0:
                    for leg in self.legs:
                        
                        leg.gaiting = False
                        leg.target = leg.position
                        leg.gaitCurrent = leg.position
                    
                    self.state = "idle" 
                    self.changedState = True

            elif self.state == "controlLeg":
                
                if self.L3Vertical != 0 or self.L3Horizontal != 0 or self.arrow != 0:
                    with self._command_lock:
                        self.legs[self.currentLeg].position.x += self.L3Horizontal * self.speed
                        self.legs[self.currentLeg].position.y += self.arrow * self.speed
                        self.legs[self.currentLeg].position.z += self.L3Vertical * self.speed
            
                        self._update_ik(self.legs[self.currentLeg])
                        self.send_command(self.desired_command)

                elif self.triangle == 1:
                    self.state = "idle"
                    self.changedState = True
            time.sleep(0.02)  # 50 Hz update for smooth robotics

    def moveLeg(self, leg, xOffset, yOffet, zOffset):
        self.L1.position.x += xOffset
        self.L1.position.y += yOffet
        self.L1.position.z += zOffset
        self._update_ik(leg)
    
    def gait(self, leg, speed, gaitEnd, gaitStart, gaitInitialTarget):

        # START gait
        if not leg.gaiting:
            
            leg.gaitCurrent = Point(leg.position.x, leg.position.y, leg.position.z)
            leg.upperMid = gaitEnd.midpoint(gaitStart)
            leg.lowerMid = gaitEnd.midpoint(gaitStart)
            leg.upperMid.z = leg.leveledZ + 50
            leg.gaiting = True

            if gaitInitialTarget == "start":
                leg.target = Point(gaitStart)
            elif gaitInitialTarget == "upperMid":
                leg.target = Point(leg.upperMid)
            elif gaitInitialTarget == "end":
                leg.target = Point(gaitEnd)
            elif gaitInitialTarget == "lowerMid":
                leg.target = Point(leg.lowerMid)


        threshold = 3

        

        # Compute increment
        increment = leg.target - leg.gaitCurrent
        increment = increment / speed

        # Update POSITION IN PLACE
        if leg.tryLevel:
            leg.position.z -= 1
            if leg.limitSwitch.is_pressed:
                leg.tryLevel = False
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = Point(gaitStart)
        else:
            leg.position.x += increment.x
            leg.position.y += increment.y
            leg.position.z += increment.z

        # Check cycle transitions
        if leg.position.distance_to(leg.target) < threshold:

            if leg.target == gaitEnd:
                if 0 == 0:
                    leg.tryLevel = False
                    leg.gaitCurrent = Point(*leg.position.__dict__.values())
                    leg.target = Point(leg.lowerMid)
                else:
                    #leg.tryLevel = True
                    leg.gaitCurrent = Point(*leg.position.__dict__.values())
                    leg.target = Point(leg.lowerMid)
                
            elif leg.target == leg.lowerMid:
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = Point(gaitStart)

            elif leg.target == gaitStart:
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = Point(leg.upperMid)
                

            elif leg.target == leg.upperMid:
                leg.gaitCurrent = Point(*leg.position.__dict__.values())
                leg.target = Point(gaitEnd)

    def body_to_leg_frame(self, leg):
    # angle of this leg relative to body x-axis
        if leg.name in ("L3", "R1"):       # front-left & front-right
            theta_deg = 45
        elif leg.name in ("L1", "R3"):     # rear-left & rear-right
            theta_deg = -45
        else:
            theta_deg = 0

        theta = math.radians(theta_deg)
        c = math.cos(theta)
        s = math.sin(theta)

        # Rotate body vector into leg frame: v_leg = R(-theta) * v_body
        x_leg =  leg.position.x * c + leg.position.y * s
        y_leg = -leg.position.x * s + leg.position.y * c
        z_leg =  leg.position.z

        return x_leg, y_leg, z_leg
                


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
                
            
    
    # -------------------------------
    # X,Y,Z Incrementing Functions:
    # -------------------------------
    def on_up_arrow_press(self):
        self.arrow = 3

    def on_down_arrow_press(self):
        self.arrow = -3

    def on_up_down_arrow_release(self):
        self.arrow = 0

    def on_left_arrow_press(self):
        self.speed -= 0.2
        if self.speed < 0.2:
            self.speed = 0.2

    def on_right_arrow_press(self):
        self.speed += 0.2
        if self.speed > 1:
            self.speed = 1

    def on_circle_press(self): 
        self.R1.position.x += 5
        self._update_ik(self.R1)
        
    def on_square_press(self): 
        self.R1.position.x -= 5
        self._update_ik(self.R1)
    
    # ----------------------------------------
    # Variable Speed Joystick Control:
    # ----------------------------------------
    def on_L3_up(self, value): 
        speed = value/-10000
        if speed > 1:
            self.L3Vertical = speed
        else:
            self.L3Vertical = 0

    def on_L3_down(self, value): 
        speed = value/-10000
        if speed * -1 > 1:
            self.L3Vertical = speed
        else:
            self.L3Vertical = 0

    def on_L3_left(self, value): 
        speed = value/10000
        if speed * -1 > 1:
            self.L3Horizontal = speed
        else:
            self.L3Horizontal = 0

    def on_L3_right(self, value): 
        speed = value/10000
        if speed > 1:
            self.L3Horizontal = speed
        else:
            self.L3Horizontal = 0

    def on_R3_up(self, value): 
        speed = value/-10000
        if speed > 1:
            self.R3Vertical = speed
        else:
            self.R3Vertical = 0

    def on_R3_down(self, value): 
        speed = value/-10000
        if speed * -1 > 1:
            self.R3Vertical = speed
        else:
            self.R3Vertical = 0

    def on_R3_left(self, value): 
        speed = value/10000
        if speed * -1 > 1:
            self.R3Horizontal = speed
        else:
            self.R3Horizontal = 0

    def on_R3_right(self, value): 
        speed = value/10000
        if speed > 1:
            self.R3Horizontal = speed
        else:
            self.R3Horizontal = 0

    # ----------------------------------------
    # Speed Manipulation For Joystick Control:
    # ----------------------------------------
    def on_triangle_press(self): pass
    def on_x_press(self): 
        self.speed -= 0.2

    # ----------------------------------------
    # Empty methods For Other Controls
    # ----------------------------------------      
    def on_x_release(self): pass
    def on_triangle_release(self): 
        if self.state == "idle" or self.state == "controlLeg":
            self.triangle = self.triangle * -1
    def on_circle_release(self): pass
    def on_square_release(self): pass
    def on_L1_press(self): pass
    def on_L1_release(self): 
        if self.state == "controlLeg":
            self.currentLeg -= 1
            
            if self.currentLeg < 0:
                self.currentLeg = len(self.legs) - 1
            print(self.currentLeg)
    def on_L2_press(self, value): pass
    def on_L2_release(self): pass
    def on_R1_press(self): 
        pass
        #self.gaiting = True
    def on_R1_release(self): 
        if self.state == "controlLeg":
            self.currentLeg += 1
            
            if self.currentLeg >= len(self.legs):
                self.currentLeg = 0
            print(self.currentLeg)
        """self.gaiting = False
        self.L1.gaiting = False
        self.L1.target = self.L1.position
        self.L1.gaitCurrent = self.L1.position"""
    def on_R2_press(self, value): pass
    def on_R2_release(self): pass
    def on_left_right_arrow_release(self): pass
    def on_L3_x_at_rest(self): pass
    def on_L3_y_at_rest(self): pass
    def on_L3_press(self): pass
    def on_L3_release(self): pass
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