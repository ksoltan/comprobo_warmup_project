import math
class Vector2D(object):
    def __init__(self, x=1, y=0, angle=None, unit="rad", magnitude=None):
        if(angle !=None and magnitude != None):
            if(unit != "rad"):
                angle = math.radians(angle % 360)
            self._x = magnitude * math.cos(angle)
            self._y = magnitude * math.sin(angle)
            return
        self._x = x
        self._y = y

    @property
    def magnitude(self):
        return math.sqrt(self._x**2 + self._y**2)

    @property
    def angle(self):
        return math.atan2(self._y, self._x)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def hat(self): # unit vector (only direction)
        return Vector2D(x=self._x / self.magnitude, y=self._y / self.magnitude)

    def normal(self):
        return Vector2D(x=self._y * -1.0, y=self._x)

    def __add__(self, other):
        return Vector2D(x=self._x + other._x, y=self._y + other._y)

    def __sub__(self, other):
        return Vector2D(x=self._x - other._x, y=self._y - other._y)

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y

    def __div__(self, scalar):
        return Vector2D(magnitude=self.magnitude / scalar, angle=self.angle)

    def __mul__(self, scalar):
        return Vector2D(magnitude=self.magnitude * scalar, angle=self.angle)

    def __repr__(self):
        return "Magnitude: {}\tAngle: {} degrees\nx: {}\t y: {}".format(self.magnitude, math.degrees(self.angle), self._x, self._y)

    def rotate(self, num_degrees):
        return Vector2D(magnitude=self.magnitude, angle=self.angle + math.radians(num_degrees))

if __name__ == "__main__":
    v1 = Vector2D(x=-1, y=1)
    v1_perp = Vector2D(x=2, y=-2)
    v2 = v1 + v1_perp
    print(v1)
    print(v1_perp)
    print(v2)
