"""base_controller controller."""
from controller import Robot
from controller import Receiver
import threading
import struct
import modules.communication as comm

orders = []
battery = 100
BASE_COORDS = {"0":[1,2,3]}


SUPERVISOR_EMITTER_CHANNEL = 0

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.setChannel(SUPERVISOR_EMITTER_CHANNEL)
receiver.enable(timestep)

state = "check_new_orders"

def update_orders():
    global battery, orders
    while True:
        if receiver.getQueueLength() > 0:
            x = receiver.getData()
            # [ "chd" , ORDER_ID , WEIGHT , DESTINATION , BASE ]
            dataList = struct.unpack("chd",x)
            receiver.nextPacket()



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
    
   
  
