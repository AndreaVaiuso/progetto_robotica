"""base_controller controller."""
from controller import Robot, Receiver, Emitter
import os
import modules.avoid_obstacles as avob
import modules.score_calculator as sccal
import threading
import random
import struct
import time
import math
import operator


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


robot = Robot()
orders = []
state_history = []
box_locked = False
anomaly_detected = False
anomaly = False

BASE_COORDS = {0: [0, 0, 0], 1: [2.17, 3.18, 0.32], 2: [-1.76, 3.18, 0.32]}
ROTATION_ANGLE_FOR_LOCK = 0
MAX_YAW = 1
MAX_PITCH = 10


def getID(name):
    x = name.split("_")
    return int(x[1])


charging = False
timestep = int(robot.getBasicTimeStep())
robot.batterySensorEnable(timestep)
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)
name = robot.getName()
drone_ID = getID(name)
receiver.setChannel(drone_ID)
current_order = []
pending_order = []
target_history = []
score_dict = {}

k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3
k_roll_p = 50
k_pitch_p = 30

altitude = 0
emergency_altitude_land = 0
roll_disturbance = 0
target_altitude = 0
target_angle = 0
posit = Coordinate(0, 0)
target_posit = Coordinate(0, 0)
precision_counter = 0
bearing = 0
stab_stack = StabilizationStack(20)
counter = 0
powerGain = 1

def checkAnomaly():
    global anomaly_detected
    while 1:
        time.sleep(500)
        anom = random.random()
        if anom > 0.9:
            anomaly_detected = True
            return


def getBaseCoords():
    return [BASE_COORDS[0][0] + drone_ID, BASE_COORDS[0][1], BASE_COORDS[0][2]]


def dPrint(string):
    perc = "--"
    try:
        perc = int(robot.batterySensorGetValue())
        perc /= 1000
        perc = str(int(perc))
    except ValueError:
        perc = "--"
    print(f"Drone ({name} [{perc}%])> {string}")


def chgState(newState, verbose=True):
    global state, state_history
    state = newState
    if verbose: dPrint(f"State changed: {newState}")
    state_history.append(state)


def chgValue(value, newValue):
    return newValue, abs(newValue - value)


def chgTarget(old_target, new_target):
    target_history.append(old_target)
    return Coordinate(new_target[0], new_target[1], new_target[2])


def getNavigationAltitude():
    return 5  # Valore reale = 10 + DRONE_ID * 2


def f2(x):
    y = math.log(abs(x) + 1, 4) / 10
    if x > 0:
        return y
    else:
        return -y


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


def limiter(value, limit):
    if abs(value) > limit:
        if value > 0:
            return limit
        else:
            return -limit
    else:
        return value


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
    # FIRST ROLL, SECOND PITCH
    a = math.radians(a)
    x0 = (x2 - x1) * math.cos(a) - (y2 - y1) * math.sin(a)
    y0 = (y2 - y1) * math.cos(a) + (x2 - x1) * math.sin(a)
    pd = - f2(x0)
    rd = f2(y0)
    pd = limiter((pd * 80), 1.2)
    rd = limiter((rd * 80), 1.2)
    return pd, rd


def get_yaw_disturbance_gain(bearing, targetAngle):
    diff = (bearing - targetAngle) % 360
    if 180 > diff > 0:
        return f(diff) * 2
    else:
        return -f(-diff + 360) * 2


def get_pitch_disturbance_gain(x1, y1, x2, y2):
    drone_position = [x1, y1]
    box_position = [x2, y2]
    distance = euc_dist(drone_position, box_position)
    return limiter(f(distance) * 10, 1.3)


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
    if value - error < target < value + error:
        return True
    return False


def clamp(val, low, high):
    if val < low:
        return low
    elif val > high:
        return high
    else:
        return val


def abort_current_order():
    global current_order
    print(current_order)
    abort_order(current_order,current=True)
    current_order = []


def score_calculator(dataList):
    global posit
    score = euc_dist(posit.getVec2d(), [dataList[4], dataList[5]])
    # Prima di lavorare con la batteria dobbiamo sapere quanta batteria ci vuole per percorrere tot metri
    return score


def abort_order(order, current=False):
    global posit, altitude
    emitter.setChannel(Emitter.CHANNEL_BROADCAST)
    if current:
        message = struct.pack("ciidddddd",b"N",int(order[1]),-1,float(order[3]),float(order[4]),float(order[5]),float(posit.x),float(posit.y),float(altitude-0.3))
    else:
        message = struct.pack("ciidddddd", b"N", int(order[1]), int(order[2]), float(order[3]), float(order[4]),
                              float(order[5]), 0.0, 0.0, 0.0)
    while emitter.send(message) != 1:
        continue


def abort_all_pending_orders():
    global orders
    while len(orders) > 0:
        abort_order(orders.pop())


def send_score(dataList):
    global pending_order
    score = score_calculator(dataList)
    # [ "ciiddd" , TYPE , DRONE_ID , ORDER_ID ,  score, 0 , 0 ]
    message = struct.pack("ciidddddd", b"S", int(drone_ID), int(dataList[1]), float(score), 0.0, 0.0, 0.0, 0.0, 0.0)
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
    sortedDict = sorted(score_dict.items(), key=operator.itemgetter(1))
    keylist = list(sortedDict)
    winner = keylist[0][0]
    if winner == drone_ID:
        orders.append(dataList)
        dPrint(f'I win! Order {dataList[1]} taken')
    if anomaly_detected or anomaly:
        abort_current_order()
    score_dict = {}


def update_orders():
    dPrint("Updating orders")
    global orders
    while True:
        if receiver.getQueueLength() > 0:  # ma dobbiamo distinguere fra due tipi di dati in arrivo , i nuovi ordini, e i punteggi
            if anomaly_detected or anomaly: return
            dPrint("Message received")
            x = receiver.getData()
            # [ [0] "N" , [1] ORDER_ID , [2] BASE , [3] WEIGHT , [4] DESTINATION_x , [5] DESTINATION_y , [6] ALT_BASE_x , [7] ALT_BASE_y ]
            # [ [0] "S" , [1] DRONE_ID , [2] ORDER_ID ,  [3] score, 0 , 0 , 0 , 0 ]
            dataList = struct.unpack("ciidddddd", x)
            if dataList[0].decode('utf-8') == 'N':
                dPrint(
                    f'New order received: [ ID:{dataList[1]}, BASE:{dataList[2]}, x:{dataList[4]}, y:{dataList[5]}, pkx:{dataList[6]}, pky:{dataList[7]} ], sending score...')
                send_score(dataList)
            elif dataList[0].decode('utf-8') == 'S':
                dPrint(f"Score arrived from DRONE: {dataList[1]}")
                if dataList[2] == pending_order[1]:
                    score_dict[dataList[1]] = dataList[3]
            receiver.nextPacket()

def getPickupPoint():
    global target_posit, current_order
    if current_order[2] != -1:
        return [BASE_COORDS[current_order[2]][0],BASE_COORDS[current_order[2]][1],BASE_COORDS[current_order[2]][2]]
    else:
        return [current_order[6],current_order[7],current_order[8]]

th = threading.Thread(target=update_orders, args=())
ag = threading.Thread(target=checkAnomaly, args=())
th.start()
ag.start()

drone_distance_sensor_upper = robot.getDevice("upper sensor")
drone_distance_sensor_upper.enable(timestep)
drone_distance_sensor_lower = robot.getDevice("lower sensor")
drone_distance_sensor_lower.enable(timestep)
drone_distance_sensor_front = robot.getDevice("front sensor")
drone_distance_sensor_front.enable(timestep)
drone_distance_sensor_right = robot.getDevice("right sensor")
drone_distance_sensor_right.enable(timestep)
drone_distance_sensor_left = robot.getDevice("left sensor")
drone_distance_sensor_left.enable(timestep)
drone_distance_sensor_lock= robot.getDevice("box sensor")
drone_distance_sensor_lock.enable(timestep)
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

check = True

chgState("check_new_orders")

while robot.step(timestep) != -1:
    pitch_disturbance = 0
    yaw_disturbance = 0
    target_angle = 0
    t = robot.getTime()
    roll = drone_imu.getRollPitchYaw()[0] + math.pi / 2
    pitch = drone_imu.getRollPitchYaw()[1]
    altitude = drone_gps.getValues()[1]
    roll_acceleration = drone_gyroscope.getValues()[0]
    pitch_acceleration = drone_gyroscope.getValues()[1]
    bearing, bearing_velocity = chgValue(bearing, get_bearing_in_degrees(drone_compass.getValues()))
    bearing_velocity *= 333
    posit.y, vel_y = chgValue(posit.y, drone_gps.getValues()[0])
    posit.x, vel_x = chgValue(posit.x, drone_gps.getValues()[2])
    x0 = vel_x * math.cos(math.radians(bearing)) - vel_y * math.sin(math.radians(bearing))
    y0 = vel_y * math.cos(math.radians(bearing)) + vel_x * math.sin(math.radians(bearing))
    vel_x = x0
    vel_y = y0
    drone_velocity = math.sqrt(pow(vel_y, 2) + pow(vel_x, 2)) * 30000
    stab = abs((roll_acceleration + pitch_acceleration) * 100)
    stab_stack.pushIntoStabStack(stab)
    left_sensor_value = drone_distance_sensor_left.getValue()
    right_sensor_value = drone_distance_sensor_right.getValue()
    front_sensor_value = drone_distance_sensor_front.getValue()
    upper_sensor_value = drone_distance_sensor_upper.getValue()
    lower_sensor_value = drone_distance_sensor_lower.getValue()
    ########### definizione della funzione avoid_obstacle_full()###########
    #### def avoid_obstacles_full(left_sensor, right_sensor, upper_sensor, lower_sensor, front_sensor)####

    if anomaly_detected: chgState("drone_anomaly_detected")

    if state == "check_new_orders":
        if len(orders) != 0:
            charging = False
            current_order = orders.pop()
            target_posit = chgTarget(target_posit,getPickupPoint())
            chgState("reach_quota")
        else:
            charging = True
            target_posit = chgTarget(target_posit, getBaseCoords())
            powerGain = 0
    elif state == "goto_recharge_battery":
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        counter += 1
        if counter > 200:
            target_altitude = 0
            if near(altitude, target_altitude, error=0.1):
                counter = 0
                chgState("check_new_orders", verbose=False)

    elif state == "reach_quota":
        powerGain = 1
        if current_order[2] == -1:
            target_altitude = getNavigationAltitude()
        else:
            target_altitude = 1
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        if avob.avoid_obstacles_full(None, None, upper_sensor_value, None, None):
            chgState('avoid_obstacles')
        elif near(altitude, target_altitude):
            dPrint("Reached target altitude, moving near box")
            chgState("go_near_box")

    elif state == "go_near_box":
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        pitch_disturbance = get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        if avob.avoid_obstacles_full(left_sensor_value, right_sensor_value, upper_sensor_value, lower_sensor_value,
                                     front_sensor_value):
            chgState('avoid_obstacles')
        elif euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 0.4:
            chgState("stabilize_on_position")

    elif state == "stabilize_on_position":
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        if stab_stack.isStable(posit, target_posit):
            dPrint("Stabilized")
            if not drone_magnetic.isLocked():
                chgState('lock_box')
            else:
                chgState('land_on_delivery_station')

    elif state == "lock_box":
        target_altitude = target_posit.z
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x, target_posit.y, bearing)
        if drone_distance_sensor_lock.getValue() < 0.05:
            dPrint("locking box")
            drone_magnetic.lock()
            if drone_magnetic.isLocked():
                target_posit = chgTarget(target_posit, [current_order[4], current_order[5], 0.3])
                chgState("reach_nav_altitude")

    elif state == "reach_nav_altitude":
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, getPickupPoint()[0], getPickupPoint()[1], bearing)
        counter += 1
        if counter > 100:
            target_altitude = getNavigationAltitude()
            target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
            yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
            if avob.avoid_obstacles_full(None, None, upper_sensor_value, None, None):
                chgState('avoid_obstacles')
            elif near(altitude, target_altitude, error=0.2):
                chgState("reach_destination")

    elif state == "reach_destination":
        pitch_disturbance = get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        if avob.avoid_obstacles_full(left_sensor_value, right_sensor_value, upper_sensor_value, lower_sensor_value,
                                     front_sensor_value):
            chgState('avoid_obstacles')
        elif euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 2:
            chgState("stabilize_on_position")

    elif state == "avoid_obstacles":
        # code
        pass

    elif state == "land_on_delivery_station":
        target_altitude = 0
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x, target_posit.y, bearing)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        if drone_distance_sensor_lower.getValue() < 1.5:
            target_altitude = altitude
            counter = 0
            chgState("detach_box")

    elif state == "detach_box":
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        counter += 1
        if counter > 100:
            while drone_magnetic.isLocked(): 
                drone_magnetic.unlock()
            target_posit = chgTarget(target_posit,getBaseCoords())
            counter = 0
            if anomaly:
                abort_current_order()
                counter = 0
                chgState("emergency_shutdown")
            else: 
                current_order = []
                chgState("go_back_home")

    elif state == "go_back_home":
        target_altitude = getNavigationAltitude()
        pitch_disturbance = get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        if avob.avoid_obstacles_full(left_sensor_value, right_sensor_value, upper_sensor_value, lower_sensor_value,
                                     front_sensor_value):
            chgState('avoid_obstalces')
        elif euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 0.4:
            chgState("stabilize_before_land_on_base")
        pass

    elif state == "stabilize_before_land_on_base":
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)

        if stab_stack.isStable(posit, target_posit):
            chgState("land_on_base")

    elif state == "land_on_base":
        target_altitude = target_posit.z + 1
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        if avob.avoid_obstacles_full(None, None, None, lower_sensor_value, None):
            chgState('avoid_obstacle')
        elif near(altitude, target_altitude, error=0.1):
            dPrint("I'm at home!")
            chgState("goto_recharge_battery")

    elif state == "emergency_shutdown":
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        counter += 1
        if counter > 50:
            pitch_disturbance = 1
            if counter > 200:
                pitch_disturbance = 0
                target_altitude = 0
                if drone_distance_sensor_lower.getValue() < 0.1:
                    powerGain = 0
                    counter = 0
                    chgState("deactivate")

    elif state == "drone_anomaly_detected":
        anomaly_detected = False
        anomaly = True
        target_posit = chgTarget(target_altitude,[posit.x,posit.y,0])
        chgState("stabilize_on_position")

    elif state == "deactivate":
        target_altitude = emergency_altitude_land
        if near(altitude,emergency_altitude_land):
            powerGain = 0
            abort_all_pending_orders()
    else:
        dPrint("ERROR, UNRECOGNIZED STATE:", state)
        break
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
    drone_front_left_motor.setVelocity(front_left_motor_input * powerGain)
    drone_front_right_motor.setVelocity(-front_right_motor_input * powerGain)
    drone_rear_left_motor.setVelocity(-rear_left_motor_input * powerGain)
    drone_rear_right_motor.setVelocity(rear_right_motor_input * powerGain)
