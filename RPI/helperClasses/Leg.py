from helperClasses.Point import Point

class Leg:
    def __init__(self, name: str, point: Point, limit_pin: int):
        
        self.name = name
        self.position = point
        self.target = self.position
        self.gaitCurrent = self.position
        self.gaiting = False
        self.leveledZ = -60
        self.tryLevel = False

    def __repr__(self):
        return f"Leg(name='{self.name}', position={self.position})"