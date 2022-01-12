"""base_controller controller."""
from controller import Robot,Receiver,Emitter 
import threading
import random
import struct
import modules.communication as comm
import time
import math
import operator
from controller import Supervisor


robot = Supervisor()
box_position = robot.getFromDef("box").getField("translation").getSFVec3f()
orders = []
battery = 100
BASE_COORDS = {"0":[1,2,3]}

timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)
name = robot.getName()
canale_di_ricezione =int(name[-1])
receiver.setChannel(canale_di_ricezione)
state = "check_new_orders"
current_order =''
score_dict={}

def gen_yaw_disturbance(baring, target_angle, error_disturbance):
        return ((math.log(abs(bearing-target_angle), 4 )-math.log(error_disturbance, 4)))/10

def get_bearing_in_degrees(values):
    rad = math.atan2(values[0],values[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing

def near(value,target,error=0.5):
    if value - error < target and value + error > target:
        return True
    return False

def clamp(val,low,high):
    if val < low:
        return low
    elif val > high:
        return high
    else: return val

def score_calculator(dataList):
    return canale_di_ricezione 

def send_score(dataList):
    score = score_calculator(dataList)
    # fare la funzione 
    #ci serve anche ORDER_ID per capire quello score a cosa si riferisce
    message = struct.pack("chd",'S', name, dataList[1],  score) #dataList è una tupla
    emitter.setChannel(Emitter.CHANNEL_BROADCAST) #dovrebbe inviare su tutti i canali cosi
    while emitter.send(message) != 1 : #se restituisce 0 significa che la coda di invio era piena e lo deve rifare
        print(f' sono il robot {name}, non sono riuscito a mandare il punteggio dello ordine {dataList[1]}, coda di invio piena')  
    current_order = dataList[1]
    print(f'sono il robot {name} ho mandato il punteggio dello ordine {current_order}')  
    score_dict[name]= score
    th = threading.Thread(target=make_topN, args=dataList)
    th.start() 

def make_topN(dataList):
    global score_dict,name  
    time.sleep(5)
    sortedDict = sorted(score_dict.items(), key=operator.itemgetter(1))
    keylist=list(sortedDict)
    winner= keylist[0]
    if winner == name:
        orders.append(dataList) 
        print(f' il robot {name} ha preso lo ordine {dataList[1]}')
    score_dict={}

def update_orders():
    global battery, orders
    
    while True:
        if receiver.getQueueLength() > 0: #ma dobbiamo distinguere fra due tipi di dati in arrivo , i nuovi ordini, e i punteggi
            x = receiver.getData()
            # [ "chd" , TYPE, ORDER_ID , WEIGHT , DESTINATION , BASE ]
            dataList = struct.unpack("chd",x) #chd è il formato di compressione
            if dataList[0] == 'NO':
                send_score(dataList)
            # [ "chd" , TYPE, DRONE_ID ,ORDER_ID, score ]
            elif dataList[0] == 'S':
                if dataList[2] == current_order:
                    score_dict[dataList[1]]=dataList[3]
                          
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
drone_camera_roll_motor = robot.getDevice("camera roll")
drone_camera_pitch_motor = robot.getDevice("camera pitch")
drone_front_left_motor = robot.getDevice("front left propeller")
drone_front_right_motor = robot.getDevice("front right propeller")
drone_rear_left_motor = robot.getDevice("rear left propeller")
drone_rear_right_motor = robot.getDevice("rear right propeller")
motors = [drone_front_left_motor,drone_front_right_motor,drone_rear_left_motor,drone_rear_right_motor]

for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1)

k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3
k_roll_p = 50
k_pitch_p = 30

state = "test1"

x1,y1=[0,0]
x2,y2=[0,0]


while robot.step(timestep) != -1:
    t = robot.getTime()
    roll = drone_imu.getRollPitchYaw()[0] + math.pi / 2
    pitch = drone_imu.getRollPitchYaw()[1]
    altitude = drone_gps.getValues()[1]
    roll_acceleration = drone_gyroscope.getValues()[0]
    pitch_acceleration = drone_gyroscope.getValues()[1]
    bearing = get_bearing_in_degrees(drone_compass.getValues())

    roll_disturbance = 0
    pitch_disturbance = 0
    yaw_disturbance = 0
    target_altitude = 0    
    error_disturbance = 1 # il cono di disturbance deve essere il più piccolo possibile 

    if state == "test1":
        print(f"State : {state}")
        target_altitude = 1.5
        if near(altitude,target_altitude):
            print("La mia posizione è: ",drone_gps.getValues())
            x1,y1 = [box_position[0],-box_position[2]] # riportare valore negativo della y per avere lo stesso riferimento della bussola 
            x2,y2 = [0,0]
            state = "test2"
    elif state == "test2":
        print(f"State : {state}")

        target_altitude =1.5
        target_angle=0        
        print(f'bearing : {bearing}')
        if (x1>0 and y1 >0) or (x1>0 and y1<0):
            target_angle = (math.degrees(math.atan((y2-y1)/(x2-x1))))
        elif (x1<0 and y1<0) or (x1<0 and y1>0):
            target_angle = 180+(math.degrees(math.atan((y2-y1)/(x2-x1))))
        if target_angle<0:
            target_angle =  360 + target_angle
        print(f"target_angle: {target_angle}")
        if target_angle- error_disturbance<bearing and target_angle+ error_disturbance> bearing :
            #fare la funzione dinamica della yaw_disturbance e decidere se girare dx o sx
            print("Rotazione Complatata !")          
        
        elif (bearing>target_angle + error_disturbance) or (bearing<target_angle - error_disturbance):
            
            next_bearing= (bearing+180)%360
            
            if bearing< next_bearing and bearing:
                if bearing<= target_angle and next_bearing> target_angle :
                    yaw_disturbance = -gen_yaw_disturbance(bearing,target_angle,error_disturbance)
                else:
                    yaw_disturbance = gen_yaw_disturbance(bearing,target_angle,error_disturbance)
            else:
                if next_bearing<= target_angle and bearing> target_angle :
                    yaw_disturbance = gen_yaw_disturbance(bearing,target_angle,error_disturbance)
                else:
                    yaw_disturbance = -gen_yaw_disturbance(bearing,target_angle,error_disturbance)
            

    elif state=='test3':
        print(f"State : {state}")
        target_altitude = 1.5
        print(f'braring : {bearing} rotazione completata')         
    elif state== "check_new_orders":
        if len(orders) != 0:
            state = "move_near_base"
        else: continue
    elif state== "move_near_base":
        #code
        pass
    elif state== "land_on_box":
        #code
        pass
    elif state== "lock_box":
        #code
        pass
    elif state== "reach_quote":
        #code
        pass
    elif state== "reach_destination":
        #code
        pass
    elif state== "avoid_obstacles":
        #code
        pass
    elif state== "land_on_delivery_station":
        #code
        pass 
    elif state== "unlock_box":
        #code
        pass       
    elif state== "go_back_home":
        #Ricordarsi di eliminare l'ordine dalla lista orders
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
    drone_front_left_motor.setVelocity(front_left_motor_input)
    drone_front_right_motor.setVelocity(-front_right_motor_input)
    drone_rear_left_motor.setVelocity(-rear_left_motor_input)
    drone_rear_right_motor.setVelocity(rear_right_motor_input)
    
   
  
