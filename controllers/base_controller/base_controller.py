"""base_controller controller."""
from controller import Robot,Receiver,Emitter 
import threading
import random
import struct
import modules.communication as comm
import time
import math
import operator



robot = Robot()


orders = [['','','','','','1']]
battery = 100
BASE_COORDS = {"0":[0,0],"1":[2.17,3.18],"2":[-1,76,3.18]}

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



def f(x):
    return math.log(x+1,4)/10
def euc_dist(drone_pos, dest_pos):
    return math.sqrt(math.pow((drone_pos[0]-dest_pos[0]),2)+math.pow((drone_pos[1]-dest_pos[1]),2))

def get_subtraction(bearing, target_angle):
    if bearing>targetAngle:
       return  bearing-targetAngle
    else:
        return targetAngle-bearing
        
def get_target_angle(x1,x2,y1,y2):
    x0 = x2-x1
    y0 = y2-y1
    if x0>=0 and y0>=0:
        return 270 + math.degrees(math.atan((y2-y1)/(x2-x1)))
    elif x0>=0 and y0<0:
        return 270 + math.degrees(math.atan((y2-y1)/(x2-x1)))
    elif x0<0 and y0>=0:
        return 90 + math.degrees(math.atan((y2-y1)/(x2-x1)))
    elif x0<0 and y0<0:
        return 90 + math.degrees(math.atan((y2-y1)/(x2-x1)))

def get_yaw_disturbance_gain(bearing,targetAngle):
    diff = (bearing-targetAngle) %360
    print("DIFF:",diff)
    if diff < 180 and diff > 0:
        return f(diff)
    else :
        return -f(-diff+360)

def get_pitch_disturbance_gain(drone_x, drone_y, box_x, box_y):
    drone_position=[drone_x, drone_y]
    box_position = [box_x, box_y]
    distance = euc_dist(drone_position,box_position)
    if distance < 0.1 :
        return f(distance)
    else:
        return -f(distance)
    
    
def gen_yaw_disturbance(bearing,maxYaw, target_angle):
    g = get_yaw_disturbance_gain(bearing,targetAngle)
    print("Bearing: ",bearing)
    print("Target Angle:", targetAngle)
    print("Gain:", g)
    print("Yaw:", yaw_disturbance)
    return maxYaw * g


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
    message = struct.pack("ssss",'S'.encode('utf-8'), name.encode('utf-8'), dataList[1],  score.encode('utf-8')) #dataList è una tupla
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
    print("sto verificando un ordine")
    global battery, orders
    
    while True:
        if receiver.getQueueLength() > 0: #ma dobbiamo distinguere fra due tipi di dati in arrivo , i nuovi ordini, e i punteggi
            x = receiver.getData()
            print(x)
            # [ "chd" , TYPE, ORDER_ID , WEIGHT , DESTINATION_x, DESTINATION_y , BASE ]
            dataList = struct.unpack("ssssss",x) 
            if dataList[0].decode('utf-8') == 'NO':
                send_score(dataList)
                print("Nuovo ordine  arrivato ! Score inviato ")
            # [ "chd" , TYPE, DRONE_ID ,ORDER_ID, score ]
            elif dataList[0].decode('utf-8') == 'S':
                print('è arrivato il punteggio di altro robot ')
                if dataList[2].decode('utf-8') == current_order:
                    score_dict[dataList[1].decode('utf-8')] = dataList[3].decode('utf-8')
                          
            receiver.nextPacket()

#th = threading.Thread(target=update_orders, args=())
#th.start()

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

#state = "test1"
maxYaw = 1
maxPitch= 10
roll_disturbance = 0
target_altitude = 0  
target_angle=0   
#error_disturbance = 0.5 # il cono di disturbance deve essere il più piccolo possibile 


while robot.step(timestep) != -1:
    pitch_disturbance = 0
    yaw_disturbance = 0
   
    t = robot.getTime()
    roll = drone_imu.getRollPitchYaw()[0] + math.pi / 2
    pitch = drone_imu.getRollPitchYaw()[1]
    altitude = drone_gps.getValues()[1]
    roll_acceleration = drone_gyroscope.getValues()[0]
    pitch_acceleration = drone_gyroscope.getValues()[1]
    bearing = get_bearing_in_degrees(drone_compass.getValues())
    y1,x1 = [drone_gps.getValues()[0],drone_gps.getValues()[2]]
    print(state)
    if current_order =='':
            y2,x2=[0,0]
    else: 
        x2,y2 = BASE_COORDS[current_order[5]] #decode
    targetAngle =get_target_angle(x1,x2,y1,y2)

   
    
    if state == "test" :
       pass
    if state == "test1":
       pass    
      
    elif state== "check_new_orders":
        if len(orders) != 0:
            current_order = orders.pop()
            state = "catch_quote"
        else: 
            print("NOT ORDERS")
            
    elif state== "catch_quote":
        target_altitude = 1
        if near(altitude,target_altitude):
            print("Altezza raggiunta")
            state = "drone_rotation"
        else:
            print("NOT QOUTE")

    elif state=='drone_rotation':
        yaw_disturbance =gen_yaw_disturbance(bearing, maxYaw, target_angle)
        diff = get_subtraction(bearing, target_angle)
        print(f'difference: {diff}')
        if  diff <5:
            state = 'move_near_base'
        else:
            print("NOT ROTATION")
    
    elif state == 'move_near_base':
        diff = get_subtraction(bearing, target_angle)
        if diff>5:
            state = 'drone_rotation'
            
        else:
            pitch_disturbance = -maxPitch*get_pitch_disturbance_gain(x1,y1,x2,y2)
            print(euc_dist([x1,y1], [x2,y2]), pitch_disturbance)
            if euc_dist([x1,y1], [x2,y2])< 0.1:
                state ='land_on_box'
            else:
                print("NOT ARRIVED")            
    
    elif state== "land_on_box":
        
        if euc_dist([x1,y1],[x2,y2]):
            state = 'move_near_base'
        else: 
            target_altitude = 0.3
            state = 'lock_box'
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
    
   
  
