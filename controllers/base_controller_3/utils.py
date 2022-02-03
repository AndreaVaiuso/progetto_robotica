import math

class StabilizationStack:
    def __init__(self, toll):
        self.stab_toll = toll
        self.stabilization_stack = [999] * toll

    def rotate(self, n):
        return self.stabilization_stack[n:] + self.stabilization_stack[:n]

    def pushIntoStabStack(self, value):
        self.stabilization_stack = self.rotate(1)
        self.stabilization_stack[-1] = value

    def getStabValue(self):
        return sum(self.stabilization_stack)

    def isStable(self, pos, target, tollerance=2):
        dist = euc_dist(pos.getVec2d(), target.getVec2d())
        s = sum(self.stabilization_stack)
        if s < tollerance and dist < 0.1: return True
        return False

    def resetStabStack(self):
        self.stabilization_stack = [999] * self.stab_toll

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

def getID(name):
    x = name.split("_")
    return int(x[1])