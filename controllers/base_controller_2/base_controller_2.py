"""base_controller controller."""
from controller import Robot, Receiver, Emitter
import threading
import random
import struct
import modules.communication as comm
import time
import math
import operator

class StabilizationStack:
    def __init__(self,toll):
        self.stab_toll = toll
        self.stabilization_stack = [999] * toll
    def rotate(self, n):
        return self.stabilization_stack[n:] + self.stabilization_stack[:n]
    def pushIntoStabStack(self, value):
        self.stabilization_stack = self.rotate(1) 
        self.stabilization_stack[-1] = value
    def getStabValue(self):
        return sum(self.stabilization_stack)
    def isStable(self,tollerance=2):
        s = sum(self.stabilization_stack)
        if s > tollerance: return False
        else: return True
    def resetStabStack(self):
        self.stabilization_stack = [999] * self.stab_toll

class Coordinate:
    x = 0
    y = 0
    def __init__(self, x, y) -> None:
        self.x = int(x)
        self.y = int(y)

    def getVec2d(self) -> list:
        return [self.x, self.y]


robot = Robot()

orders = []
state_history = []
battery = 100
box_locked = False

BASE_COORDS = {0: [0, 0], 1: [2.17, 3.18], 2: [-1.76, 3.18]}
ROTATION_ANGLE_FOR_LOCK = 0
MAX_YAW = 1
MAX_PITCH = 10

def getID(name):
    x = name.split("_")
    return int(x[1])

timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)
name = robot.getName()
drone_ID = getID(name)
receiver.setChannel(drone_ID)
current_order = []
pending_order = []
score_dict = {}

def dPrint(string):
    print(f"Drone ({name})> {string}")

def chgState(newState, verbose=True):
    global state, state_history
    state = newState
    if verbose: dPrint(f"State changed: {newState}")
    state_history.append(state)

def chgValue(value,newValue):
    return newValue, abs(newValue-value)

def f2(x):
    y = math.log(abs(x) + 1, 4) / 10
    if x > 0:
        return y
    else: return -y

def f(x):
    y = math.log(x + 1, 4) / 10
    return y


def euc_dist(drone_pos, dest_pos):
    return math.sqrt(math.pow((drone_pos[0] - dest_pos[0]), 2) + math.pow((drone_pos[1] - dest_pos[1]), 2))


def dist1d(drone_pos, dest_pos):
    return dest_pos[0] - drone_pos[0]


def get_subtraction(bearing, target_angle):
    if bearing > target_angle:
        return bearing - target_angle
    else:
        return target_angle - bearing

def limiter(value,limit):
    if abs(value) > limit:
        if value > 0:
            return limit
        else: return -limit
    else: return value

def get_target_angle(x1, x2, y1, y2):
    x0 = x2 - x1
    y0 = y2 - y1
    if x0 >= 0 and y0 >= 0:
        return 270 + math.degrees(math.atan((y2 - y1) / (x2 - x1)))
    elif x0 >= 0 and y0 < 0:
        return 270 + math.degrees(math.atan((y2 - y1) / (x2 - x1)))
    elif x0 < 0 and y0 >= 0:
        return 90 + math.degrees(math.atan((y2 - y1) / (x2 - x1)))
    elif x0 < 0 and y0 < 0:
        return 90 + math.degrees(math.atan((y2 - y1) / (x2 - x1)))

def get_stabilization_disturbance(x1, y1, x2, y2, a):
    #FIRST ROLL, SECOND PITCH
    a = math.radians(a)
    x0 = (x2 - x1) * math.cos(a) - (y2 - y1) * math.sin(a)
    y0 = (y2 - y1) * math.cos(a) + (x2 - x1) * math.sin(a)
    pd = - f2(x0)
    rd = f2(y0)
    pd = limiter((pd * MAX_PITCH * 8),1.2)
    rd = limiter((rd * MAX_PITCH * 8),1.2)
    return pd, rd

def get_yaw_disturbance_gain(bearing, targetAngle):
    diff = (bearing - targetAngle) % 360
    if diff < 180 and diff > 0:
        return f(diff)
    else:
        return -f(-diff + 360)


def get_pitch_disturbance_gain(x1, y1, x2, y2):
    drone_position = [x1, y1]
    box_position = [x2, y2]
    distance = euc_dist(drone_position, box_position)
    if distance > 10: return 1
    if distance < 0:
        return f(distance)
    else:
        return -f(distance)


def gen_yaw_disturbance(bearing, maxYaw, target_angle):
    g = get_yaw_disturbance_gain(bearing, target_angle)
    return maxYaw * g


def get_bearing_in_degrees(values):
    rad = math.atan2(values[0], values[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing


def near(value, target, error=0.5):
    if value - error < target and value + error > target:
        return True
    return False


def clamp(val, low, high):
    if val < low:
        return low
    elif val > high:
        return high
    else:
        return val


def score_calculator(dataList):
    global posit
    score = euc_dist(posit.getVec2d(), [dataList[4], dataList[5]])
    # Prima di lavorare con la batteria dobbiamo sapere quanta batteria ci vuole per percorrere tot metri
    return score


def send_score(dataList):
    global pending_order
    score = score_calculator(dataList)
    # [ "ciiddd" , TYPE , DRONE_ID , ORDER_ID ,  score, 0 , 0 ]
    message = struct.pack("ciiddd", b"S", int(drone_ID), int(dataList[1]), float(score), 0.0, 0.0)
    emitter.setChannel(Emitter.CHANNEL_BROADCAST)
    while emitter.send(message) != 1:
        dPrint(f'Waiting queue for sending message')
    pending_order = dataList
    score_dict[drone_ID] = score
    th = threading.Thread(target=make_topN, args=[dataList])
    th.start()


def make_topN(dataList):
    global score_dict, name
    time.sleep(5)
    print(score_dict)
    sortedDict = sorted(score_dict.items(), key=operator.itemgetter(1))
    keylist = list(sortedDict)
    print(keylist)
    winner = keylist[0][0]
    print(winner)
    if winner == drone_ID:
        orders.append(dataList)
        dPrint(f'I win! Order {dataList[1]} taken')
    score_dict = {}


def update_orders():
    dPrint("Updating orders")
    global battery, orders
    while True:
        if receiver.getQueueLength() > 0:  # ma dobbiamo distinguere fra due tipi di dati in arrivo , i nuovi ordini, e i punteggi
            dPrint("Message received")
            x = receiver.getData()
            # [ [0] "N" , [1] ORDER_ID , [2] BASE , [3] WEIGHT , [4] DESTINATION_x , [5] DESTINATION_y ]
            # [ [0] "S" , [1] DRONE_ID , [2] ORDER_ID ,  [3] score, 0 , 0 ]
            dataList = struct.unpack("ciiddd", x)
            if dataList[0].decode('utf-8') == 'N':
                dPrint(
                    f'New order received: [ ID:{dataList[1]}, BASE:{dataList[2]}, x:{dataList[4]}, y:{dataList[5]} ], sending score...')
                send_score(dataList)
            elif dataList[0].decode('utf-8') == 'S':
                dPrint(f"Score arrived from DRONE: {dataList[1]}")
                if dataList[2] == pending_order[1]:
                    score_dict[dataList[1]] = dataList[3]
            receiver.nextPacket()


th = threading.Thread(target=update_orders, args=())
th.start()

drone_camera = robot.getDevice("camera")
drone_camera.enable(timestep)
drone_imu = robot.getDevice("inertial unit")
drone_imu.enable(timestep)
drone_gps = robot.getDevice("gps")
drone_gps.enable(timestep)
drone_compass = robot.getDevice("compass")
drone_compass.enable(timestep)
drone_gyroscope = robot.getDevice("gyro")
drone_gyroscope.enable(timestep)
drone_magnetic = robot.getDevice("magnetic")
drone_magnetic.enablePresence(timestep)
drone_camera_roll_motor = robot.getDevice("camera roll")
drone_camera_pitch_motor = robot.getDevice("camera pitch")
drone_front_left_motor = robot.getDevice("front left propeller")
drone_front_right_motor = robot.getDevice("front right propeller")
drone_rear_left_motor = robot.getDevice("rear left propeller")
drone_rear_right_motor = robot.getDevice("rear right propeller")

motors = [drone_front_left_motor, drone_front_right_motor, drone_rear_left_motor, drone_rear_right_motor]

for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1)

k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3
k_roll_p = 50
k_pitch_p = 30

roll_disturbance = 0
target_altitude = 0
target_angle = 0
posit = Coordinate(0, 0)
target_posit = Coordinate(0, 0)
precision_counter = 0
bearing = 0
stab_stack = StabilizationStack(20)

chgState("check_new_orders")

while robot.step(timestep) != -1:
    pitch_disturbance = 0
    yaw_disturbance = 0
    target_angle = 0
    powerGain = 1
    t = robot.getTime()
    roll = drone_imu.getRollPitchYaw()[0] + math.pi / 2
    pitch = drone_imu.getRollPitchYaw()[1]
    altitude = drone_gps.getValues()[1]
    roll_acceleration = drone_gyroscope.getValues()[0]
    pitch_acceleration = drone_gyroscope.getValues()[1]
    bearing, bearing_velocity = chgValue(bearing,get_bearing_in_degrees(drone_compass.getValues()))
    bearing_velocity *= 333
    posit.y, vel_y = chgValue(posit.y,drone_gps.getValues()[0])
    posit.x, vel_x = chgValue(posit.x,drone_gps.getValues()[2])
    x0 = vel_x * math.cos(math.radians(bearing)) - vel_y * math.sin(math.radians(bearing))
    y0 = vel_y * math.cos(math.radians(bearing)) + vel_x * math.sin(math.radians(bearing))
    vel_x = x0
    vel_y = y0
    drone_velocity = math.sqrt(pow(vel_y,2)+pow(vel_x,2)) * 30000
    stab = abs((roll_acceleration+pitch_acceleration)*100)
    stab_stack.pushIntoStabStack(stab)

    if state == "check_new_orders":
        if len(orders) != 0:
            current_order = orders.pop()
            target_posit.x = BASE_COORDS[current_order[2]][0]
            target_posit.y = BASE_COORDS[current_order[2]][1]
            chgState("reach_quota")
        else:
            target_posit.x = BASE_COORDS[0][0] + drone_ID
            target_posit.y = BASE_COORDS[0][1]
            chgState("go_to_recharge", verbose=False)

    elif state == "go_to_recharge":
        chgState("check_new_orders", verbose=False)

    elif state == "reach_quota":
        target_altitude = 1
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        if near(altitude, target_altitude):
            dPrint("Reached target altitude, moving near box")
            chgState("go_near_box")

    elif state == "go_near_box":
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        target_posit.x = BASE_COORDS[current_order[2]][0]
        target_posit.y = BASE_COORDS[current_order[2]][1]
        pitch_disturbance = - MAX_PITCH * get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        if euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 0.4:
            chgState("stabilize_on_position")

    elif state == "stabilize_on_position":
        if euc_dist(posit.getVec2d(), target_posit.getVec2d()) >= 0.4:
            chgState(state_history[-2])
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x, target_posit.y, bearing)
        if stab_stack.isStable():
            dPrint("Stabilized")
            if not drone_magnetic.isLocked():
                chgState('lock_box')
            else:
                chgState('unlock_box') 

    elif state == "lock_box":
        target_altitude = 0.35
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x, target_posit.y, bearing)
        if near(altitude, target_altitude, error=0.1):
            dPrint("locking...")
            drone_magnetic.lock()
            if drone_magnetic.isLocked():
                chgState("reach_nav_altitude")

    elif state == "reach_nav_altitude":
        dPrint("locked")
        target_altitude = 5
    elif state == "reach_destination":
        # code
        pass
    elif state == "avoid_obstacles":
        # code
        pass
    elif state == "land_on_delivery_station":
        # code
        pass
    elif state == "unlock_box":
        # code
        pass
    elif state == "go_back_home":
        # Ricordarsi di eliminare l'ordine dalla lista orders
        pass

        # Compute the roll, pitch, yaw and vertical inputs.
    roll_input = k_roll_p * clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
    pitch_input = k_pitch_p * clamp(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_difference_altitude = clamp(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

    # Actuate the motors taking into consideration all the computed inputs.
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    drone_front_left_motor.setVelocity(front_left_motor_input*powerGain)
    drone_front_right_motor.setVelocity(-front_right_motor_input*powerGain)
    drone_rear_left_motor.setVelocity(-rear_left_motor_input*powerGain)
    drone_rear_right_motor.setVelocity(rear_right_motor_input*powerGain)
