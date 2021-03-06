import math

class StabilizationArray:

    def __init__(self, toll):
        self.stab_toll = toll
        self.stabilization_array = [999] * toll

    def rotate(self, n):
        return self.stabilization_array[n:] + self.stabilization_array[:n]

    def pushIntoStabArray(self, value):
        self.stabilization_array = self.rotate(1)
        self.stabilization_array[-1] = value

    def getStabValue(self):
        return sum(self.stabilization_array)

    def isStable(self, pos, target, tollerance=2):
        dist = euc_dist(pos.getVec2d(), target.getVec2d())
        s = sum(self.stabilization_array)
        if s < tollerance and dist < 0.1: return True
        return False

    def resetStabArray(self):
        self.stabilization_array = [999] * self.stab_toll

class Coordinate:
    x = 0
    y = 0
    z = 0

    def __init__(self, x, y, z=0.35) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def getVec2d(self) -> list:
        return [self.x, self.y]

    def getVec3d(self) -> list:
        return [self.x, self.y, self.z]

def euc_dist(drone_pos, dest_pos):
    return math.sqrt(math.pow((drone_pos[0] - dest_pos[0]), 2) + math.pow((drone_pos[1] - dest_pos[1]), 2))

def euc_dist3(drone_pos, dest_pos):
    return math.sqrt(math.pow((drone_pos[0] - dest_pos[0]), 2) + math.pow((drone_pos[1] - dest_pos[1]), 2) + math.pow((drone_pos[2] - dest_pos[2]), 2))


def getID(name):
    x = name.split("_")
    return int(x[1])