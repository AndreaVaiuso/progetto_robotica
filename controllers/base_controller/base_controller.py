"""base_controller controller."""
from controller import Robot,Receiver, Emitter 
import threading
import random
import struct
import modules.communication as comm

DRONE_ID = int(random.randrange(1,1000000000000000))
orders = []
battery = 100
BASE_COORDS = {"0":[1,2,3]}


SUPERVISOR_EMITTER_CHANNEL = 0
GENERAL_CHANNEL = 1


robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.setChannel(SUPERVISOR_EMITTER_CHANNEL)
receiver.enable(timestep)
state = "check_new_orders"

def score_calculator(datalist):
    pass 

def update_orders():
    global battery, orders
    while True:
        if receiver.getQueueLength() > 0:
            x = receiver.getData()
            # [ "chd" , ORDER_ID , WEIGHT , DESTINATION , BASE ]
            dataList = struct.unpack("chd",x)
            receiver.nextPacket()
        score = score_calculator(dataList) # fare la funzione 
        message = struct.pack("chd", DRONE_ID, score)
        emitter.setChannel(GENERAL_CHANNEL)
        emitter.send(message)
        



th = threading.Thread(target=update_orders, args=())
th.start()

while robot.step(timestep) != -1:
    if state== "check_new_orders":
        #code
        pass
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
        #code
        pass      
    
   
  
