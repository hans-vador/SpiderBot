import math
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

