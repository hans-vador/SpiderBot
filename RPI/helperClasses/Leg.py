from helperClasses.Point import Point

class Leg:
    def __init__(self, name: str, point: Point, limit_pin: int):
        
        self.name = name
        self.position = Point(point.x, point.y, point.z)
        self.target = Point(point.x, point.y, point.z)
        self.upperMid = Point(0, 0, 0)
        self.lowerMid = Point(0, 0, 0)
        self.gaitCurrent = Point(point.x, point.y, point.z)
        self.gaiting = False
        self.leveledZ = -60
        self.tryLevel = False

        self.gaitTarget = ("none", point)
        self.gaitPrev = ("none", point)

    def __repr__(self):
        return f"Leg(name='{self.name}', position={self.position})"