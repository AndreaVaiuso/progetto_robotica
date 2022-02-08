"""base_controller controller."""
from controller import Robot, Receiver, Emitter
import modules.avoid_obstacles as avob
import modules.score_calculator as sccal
from utils import StabilizationArray, Coordinate, euc_dist, euc_dist3, getID
import threading
import random
import struct
import time
import math
import operator

BASE_COORDS = {0: [0, 0, 0], 1: [2.17, 3.18, 0.32], 2: [-1.76, 3.18, 0.32], 3: [-5.69, 3.18, 0.32]}
ROTATION_ANGLE_FOR_LOCK = 0
MAX_YAW = 1
MAX_PITCH = 10

robot = Robot()
posit = Coordinate(0, 0)
timestep = int(robot.getBasicTimeStep())
robot.batterySensorEnable(timestep)
state_history = ["check_new_orders"]
state = "check_new_orders"
old_state = ""
orders = []
box_locked = False
anomaly_detected = False
anomaly = False
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)
name = robot.getName()
drone_ID = getID(name)
receiver.setChannel(int(drone_ID) + 1)
current_order = []
battery_for_current = ""  # batteria che mi serve per eseguire l'ordine corrente
target_history = []
score_dict = {}


def checkBattery():
    global battery_for_current, current_order, old_state, posit
    battery = robot.batterySensorGetValue()
    tempo_percorso = 0
    recharge_battery_to_reach_quote = 10
    reach_quote_to_lock_box = 30
    land_on_delivery_to_go_back_home = 30
    ostacoli = 50  # stiamo 50 secondi a schivare ostacoli in media durante un tragitto
    if current_order[2] != -1:
        distanza = euc_dist([posit.x, posit.y],
                            [current_order[4], current_order[5]]) * 2  # ogni metro impiego mediamente 1,7 s
    else:
        distanza_pacco = euc_dist([posit.x, posit.y],
                                  [current_order[6], current_order[7]])  # dalla base alla posizione del pacco
        distanza_consegna = euc_dist([current_order[6], current_order[7]],
                                     [current_order[4], current_order[5]])  # dalla posizione del pacco alla consegna
        distanza_ritorno = euc_dist([posit.x, posit.y],
                                    [current_order[4], current_order[5]])  # dalla consegna alla base
        distanza = distanza_pacco + distanza_consegna + distanza_ritorno

    if old_state == "go_back_home":
        tempo_percorso = distanza * 1.7 + reach_quote_to_lock_box + land_on_delivery_to_go_back_home + ostacoli
    else:
        tempo_percorso = distanza * 1.7 + recharge_battery_to_reach_quote + reach_quote_to_lock_box + land_on_delivery_to_go_back_home + ostacoli
    battery_for_current_temp = tempo_percorso * 50  # ogni secondo il robot perde 50J di batteria
    battery_for_current = battery_for_current_temp + ((battery_for_current_temp / 100) * 20)  # 20% in più per sicurezza
    if battery >= battery_for_current:
        return True
    else:
        return False


def checkAllBattery():
    global orders, state_history, posit
    score_all = sccal.sccal(orders, [], [], state_history, posit, getBaseCoords())
    battery_for_all = battery_for_current + score_all * 50
    return battery_for_all


def getPickupPoint():
    global target_posit, current_order
    if current_order[2] != -1:
        return [BASE_COORDS[current_order[2]][0], BASE_COORDS[current_order[2]][1], BASE_COORDS[current_order[2]][2]]
    else:
        return [current_order[6], current_order[7], current_order[8]]


def checkAnomaly():
    global anomaly_detected
    while 1:
        time.sleep(500)
        # time.sleep(60)
        # anomaly_detected = True
        # return
        anom = random.random()
        if anom > 0.8:
            anomaly_detected = True
            return


def getBaseCoords():
    return [BASE_COORDS[0][0] + drone_ID, BASE_COORDS[0][1], BASE_COORDS[0][2]]

def getBatteryPercent():
    perc = -1
    try:
        perc = int(robot.batterySensorGetValue())
        perc /= 1000
        perc = int(perc)
    except ValueError:
        perc = -1
    return perc

def dPrint(string):
    perc = str(getBatteryPercent())
    if current_order != []:
        c = 1
    else:
        c = 0
    print(f"Drone ({name} [{perc}%][{c + len(orders)}])> {string}")


def chgState(newState, verbose=True):
    global state, state_history, old_state
    state = newState
    if verbose: dPrint(f"State changed: {newState}")
    if len(state_history) > 0:
        old_state = state_history[-1]
    state_history.append(state)


def chgValue(value, newValue):
    return newValue, abs(newValue - value)


def chgTarget(old_target, new_target):
    target_history.append(old_target)
    return Coordinate(new_target[0], new_target[1], new_target[2])


def getNavigationAltitude():
    return 10 + drone_ID * 2


def f2(x):
    y = math.log(abs(x) + 1, 4) / 10
    if x > 0:
        return y
    else:
        return -y


def f(x):
    y = math.log(x + 1, 4) / 10
    return y


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
    abort_order(current_order, current=True)
    current_order = []


def abort_order(order, current=False):
    global posit, altitude
    emitter.setChannel(Emitter.CHANNEL_BROADCAST)
    if current:
        message = struct.pack("ciidddddd", b"N", int(order[1]), -1, float(order[3]), float(order[4]), float(order[5]),
                              float(posit.x), float(posit.y), float(altitude - 0.3))
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
    global orders, current_order, state_history, posit
    order_ID = dataList[1]
    score = sccal.sccal(orders, dataList, current_order, state_history, posit, getBaseCoords())
    dPrint("Score sent: " + str(score))
    message = struct.pack("ciidddddd", b"S", int(drone_ID), int(dataList[1]), float(score), 0.0, 0.0, 0.0, 0.0, 0.0)
    emitter.setChannel(Emitter.CHANNEL_BROADCAST)
    while emitter.send(message) != 1:
        dPrint(f'Waiting queue for sending message')
    # [ [0] "S" , [1] DRONE_ID , [2] ORDER_ID ,  [3] SCORE , 0 , 0 , 0 , 0 ]
    score_dict[drone_ID, order_ID] = [order_ID, score]
    time.sleep(3)
    score_for_order = []
    for i in list(score_dict.keys()):
        elem = score_dict[i]
        if elem[0] == order_ID:
            score_for_order.append([elem[1], i[0]])
    score_for_order.sort(key=lambda x: x[0])
    winner = score_for_order[0][1]
    if winner == drone_ID:
        dPrint(f'I win! Order {order_ID} taken')
        orders.append(dataList)
        if anomaly_detected or anomaly:
            abort_all_pending_orders()
    for i in list(score_dict.keys()):
        if score_dict[i][0] == order_ID:
            del score_dict[i]


def update_orders():
    dPrint("Updating orders")
    global orders, thread_index, score_dict
    while True:
        if receiver.getQueueLength() > 0:  # ma dobbiamo distinguere fra due tipi di dati in arrivo , i nuovi ordini, e i punteggi
            if anomaly_detected or anomaly: return
            dPrint("Message received")
            x = receiver.getData()
            # [ [0] "N" , [1] ORDER_ID , [2] BASE , [3] WEIGHT , [4] DESTINATION_x , [5] DESTINATION_y , [6] ALT_BASE_x , [7] ALT_BASE_y, [8] ALT_BASE_z ]
            # [ [0] "S" , [1] DRONE_ID , [2] ORDER_ID ,  [3] score, 0 , 0 , 0 , 0 ]
            dataList = struct.unpack("ciidddddd", x)
            if dataList[0].decode('utf-8') == 'N':
                dPrint(
                    f'New order received: [ ID:{dataList[1]}, BASE:{dataList[2]}, x:{dataList[4]}, y:{dataList[5]}, pkx:{dataList[6]}, pky:{dataList[7]} ], sending score...')
                scth = threading.Thread(target=send_score, args=[dataList])
                scth.start()
            elif dataList[0].decode('utf-8') == 'S':
                dPrint(f"Score for order {str(dataList[2])} arrived from DRONE: {dataList[1]}")
                score_dict[dataList[1], dataList[2]] = [dataList[2], dataList[3]]
            receiver.nextPacket()


def completeDelivery(orderID):
    message = struct.pack("ciidddddd", b"C", int(orderID), 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    emitter.setChannel(0)
    while emitter.send(message) != 1:
        dPrint(f'Waiting queue for sending message')


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
drone_distance_sensor_lock = robot.getDevice("box sensor")
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
target_posit = Coordinate(0, 0)
precision_counter = 0
bearing = 0
stab_stack = StabilizationArray(20)
counter = 0
powerGain = 0
stabilization_position = Coordinate(0, 0, 0)
count_altitude = 0
count_lock_box = 0
count_avoid=0

# chgState("check_new_orders") gia inseriti in state e state_history di default, vedi su

while robot.step(timestep) != -1:
    pitch_disturbance = 0
    yaw_disturbance = 0
    target_angle = 0
    t = robot.getTime()
    roll = drone_imu.getRollPitchYaw()[0] + math.pi / 2
    pitch = drone_imu.getRollPitchYaw()[1]
    altitude, altitude_velocity = chgValue(altitude, drone_gps.getValues()[1])
    altitude_velocity *= 9000
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
    drone_velocity = math.sqrt(pow(vel_y, 2) + pow(vel_x, 2)) * 15000
    stab = abs((roll_acceleration + pitch_acceleration) * 100)
    stab_stack.pushIntoStabArray(stab)
    left_sensor_value = drone_distance_sensor_left.getValue()
    right_sensor_value = drone_distance_sensor_right.getValue()
    front_sensor_value = drone_distance_sensor_front.getValue()
    upper_sensor_value = drone_distance_sensor_upper.getValue()
    lower_sensor_value = drone_distance_sensor_lower.getValue()

    if anomaly_detected: chgState("drone_anomaly_detected")
    if getBatteryPercent() < 3 and euc_dist3(posit.getVec3d,getBaseCoords()) > 10:
        anomaly_detected = True

    if state == "check_new_orders":
        if current_order != []:
            c = 1
        else:
            c = 0
        if (len(orders) + c) != 0:
            if current_order == []:
                current_order = orders.pop(0)
            if checkBattery():
                target_posit = chgTarget(target_posit, getPickupPoint())
                state_history = ["check_new_orders"]
                chgState("reach_quota")
            else:
                target_posit = chgTarget(target_posit, getBaseCoords())
                if old_state == "go_back_home":
                    state_history = ["check_new_orders"]
                    chgState("stabilize_before_land_on_base")
                else:
                    state_history = ["check_new_orders"]
                    chgState("recharge_battery")
        else:
            target_posit = chgTarget(target_posit, getBaseCoords())
            if old_state == "go_back_home":
                state_history = ["check_new_orders"]
                chgState("stabilize_before_land_on_base")
            elif old_state == "lock_box":
                target_altitude = 1.5
                chgState("go_back_home")

    elif state == "goto_recharge_battery":
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        counter += 1
        if counter > 200:
            target_altitude = 0
            if near(altitude, target_altitude, error=0.1):
                counter = 0
                chgState("recharge_battery")

    elif state == "recharge_battery":
        powerGain = 0
        battery = robot.batterySensorGetValue()
        if checkAllBattery() <= battery or battery >= 99900:
            chgState("check_new_orders")

    elif state == "reach_quota":
        # avoid obstacle
        if avob.avoid_obstacles_full(upper_sensor_value, None, None, None,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        powerGain = 1
        target_altitude = 1.5
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)

        if near(altitude, target_altitude):
            dPrint("Reached target altitude, moving near box")
            chgState("go_near_box")

    elif state == "go_near_box":
        # avoid obstacle
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        pitch_disturbance = get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        if euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 0.4:
            chgState("stabilize_on_position")

    elif state == "stabilize_on_position":
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        if stab_stack.isStable(posit, target_posit):
            dPrint("Stabilized")
            if old_state == "drone_anomaly_detected":
                chgState('land_on_delivery_station')
            else:
                if not drone_magnetic.isLocked():
                    chgState('lock_box')
                else:
                    chgState('land_on_delivery_station')

    elif state == "lock_box":
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        target_altitude = target_posit.z
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        if drone_distance_sensor_lock.getValue() < 0.05:
            dPrint("locking box")
            drone_magnetic.lock()
            if drone_magnetic.isLocked():
                target_posit = chgTarget(target_posit, [current_order[4], current_order[5], 0.3])
                chgState("reach_nav_altitude")
        if near(altitude, target_altitude, error=0.3):
            count_lock_box += 1
            if count_lock_box > 700:
                current_order = []
                chgState("check_new_orders")

    elif state == "reach_nav_altitude":
        # avoid obstacle
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, getPickupPoint()[0],
                                                                            getPickupPoint()[1], bearing)
        counter += 1
        if counter > 100:
            pitch_disturbance=0.5
            target_altitude = getNavigationAltitude()
            target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
            yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
            if near(altitude, target_altitude, error=0.2):
                counter = 0
                chgState("reach_destination")

    elif state == "reach_destination":
        # avoid obstacle
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        pitch_disturbance = get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        if euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 2:
            chgState("stabilize_on_position")

    elif state == "land_on_delivery_station":
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        target_altitude = target_posit.z + 0.5
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
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
            target_posit = chgTarget(target_posit, getBaseCoords())
            counter = 0
            if anomaly:
                abort_current_order()
                counter = 0
                chgState("emergency_shutdown")
            else:
                completeDelivery(current_order[1])
                current_order = []
                chgState("go_back_home")

    elif state == "go_back_home":
        # avoid obstacle
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value,
                                     right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        pitch_disturbance = get_pitch_disturbance_gain(posit.x, posit.y, target_posit.x, target_posit.y)
        target_angle = get_target_angle(posit.x, target_posit.x, posit.y, target_posit.y)
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, target_angle)
        if euc_dist(posit.getVec2d(), target_posit.getVec2d()) < 5:
            chgState("check_new_orders")

    elif state == "stabilize_before_land_on_base":
        if avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value,
                                     right_sensor_value,
                                     [drone_velocity, altitude_velocity]):
            chgState('avoid_obstacles')
        yaw_disturbance = gen_yaw_disturbance(bearing, MAX_YAW, 0)
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        if stab_stack.isStable(posit, target_posit):
            chgState("land_on_base")

    elif state == "land_on_base":
        target_altitude = target_posit.z + 1
        roll_disturbance, pitch_disturbance = get_stabilization_disturbance(posit.x, posit.y, target_posit.x,
                                                                            target_posit.y, bearing)
        if near(altitude, target_altitude, error=0.1):
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
        target_posit = chgTarget(target_altitude, [posit.x, posit.y, 0])
        chgState("stabilize_on_position")

    elif state == "deactivate":
        target_altitude = emergency_altitude_land
        if near(altitude, emergency_altitude_land):
            powerGain = 0
            abort_all_pending_orders()

    elif state == 'avoid_obstacles':
        #count_avoid+=1
        if not avob.avoid_obstacles_full(upper_sensor_value, front_sensor_value, left_sensor_value, right_sensor_value,
                                         [drone_velocity, altitude_velocity]): #and count_avoid>50:
            if state_history[-2] == ('land_on_delivery_station' or 'lock_box'):
                target_altitude = altitude + 1
                count_avoid=0
                chgState(state_history[-3])
            elif state_history[-2] == 'go_near_box':
                count_avoid=0
                chgState(state_history[-2])
            elif state_history[-2] == 'reach_nav_altitude':
                chgState('reach_destination')
            else:
                target_altitude = altitude + 1
                count_avoid=0
                chgState(state_history[-2])
        else:
            if avob.avoid_obstacles_sensor(upper_sensor_value, altitude_velocity):
                target_altitude = altitude - 0.1
                #roll_disturbance = 0.8
                string = 'upper sensor value : ' + str(upper_sensor_value)
                #dPrint(string)

            if avob.avoid_obstacles_sensor(left_sensor_value, drone_velocity):
                target_altitude += 0.2
                pitch_disturbance = 0
                roll_disturbance = -0.3
                string = 'left sensor value : ' + str(left_sensor_value)
                #dPrint(string)

            if avob.avoid_obstacles_sensor(right_sensor_value, drone_velocity):
                target_altitude += 0.2
                pitch_disturbance = 0
                roll_disturbance = 0.3
                string = 'right sensor value : ' + str(right_sensor_value)
                #dPrint(string)

            if avob.avoid_obstacles_sensor(front_sensor_value, drone_velocity):
                pitch_disturbance = -0.2
                target_altitude += 0.2
                string = 'front sensor value : ' + str(front_sensor_value)
                #dPrint(string)


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
