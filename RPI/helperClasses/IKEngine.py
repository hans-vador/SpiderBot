import math
COXA = 43
FEMUR = 60
TIBIA = 110
class IKEngine:
    def __init__(self):
        self.S1Angle = 0
        self.S2Angle = 0
        self.S3Angle = 0

    def calculate(self, name, x, y, z):
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

        
        if name == "L1" or name == "L3":
            self.S1Angle = 90 + J1 
            self.S2Angle = 90 - J2
            self.S3Angle = J3
        if name == "R1" or name == "R3":
            self.S1Angle = 90 + J1 
            self.S2Angle = 90 + J2
            self.S3Angle = 180 - J3
        

        return self.S1Angle, self.S2Angle, self.S3Angle