import time
import struct
import math
import random
import threading
from venv import create
from controller import Supervisor, Emitter

BASE_COORDS = {0:[0,0,0.3],1:[2.17,3.18,0.3],2:[-1.76,3.18,0.3]}
DESTINATIONS_COORDS = {0:[10,10], 1:[-6,25], 2:[-11,-16], 3:[35,-15]}

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
emitter = supervisor.getDevice("emitter")
emitter.setChannel(Emitter.CHANNEL_BROADCAST)


def getID(name,spl="_"):
    x = name.split(spl)
    return int(x[-1])

def euc_dist_3(drone_pos, dest_pos):
    return math.sqrt(math.pow((drone_pos[0] - dest_pos[0]), 2) + math.pow((drone_pos[1] - dest_pos[1]),2) + math.pow((drone_pos[2] - dest_pos[2]), 2))

def baseIsFree(base):
    root_node = supervisor.getRoot()
    children_field = root_node.getField("children")
    number_of_objects = children_field.getCount()
    for i in range(number_of_objects):
        element = children_field.getMFNode(i)
        el_name = element.getField("name")
        try:
            name = el_name.getSFString()
            if name.startswith("BOX_"):
                if getID(name) == base:
                    posit = element.getField("translation").getSFVec3f()
                    posit = [posit[2],posit[0],posit[1]]
                    dist = euc_dist_3(BASE_COORDS[base],posit)
                    while dist<=3:
                        return False
                else: continue
        except AttributeError:
            continue
    return True

def createOrder(orderID,weight,position,base):
    root_node = supervisor.getRoot()
    children_field = root_node.getField("children")
    message = struct.pack("ciidddddd",b"N",int(orderID),int(base),float(weight),float(position[0]),float(position[1]),0.0,0.0,0.0)
    print("Nuovo pacco>> ID:", orderID, "position: [",position[0],",",position[1],"]","base:",base)
    while emitter.send(message) != 1 :
        continue
    xp = BASE_COORDS[base][0]
    yp = BASE_COORDS[base][1]
    zp = 0.26
    children_field.importMFNode(-1,"../../protos/box.wbo") 
    node = children_field.getMFNode(-1) 
    field = node.getField("translation") 
    name = node.getField("name")
    field.setSFVec3f([yp,zp,xp])
    name.setSFString("BOX_"+str(orderID)+"_B_"+str(base))

counter_id = 0

state = "create_order"
dest = 0
base = 0

ct = 0

while supervisor.step(timestep) != -1:
    if state == "create_order":
        dest = random.randint(0,(len(DESTINATIONS_COORDS) - 1))
        state = "wait_for_free_base"
    if state == "wait_for_free_base":
        base = random.randint(1,(len(BASE_COORDS) - 1))
        ct += 1
        if ct>500:
            ct = 0
            if baseIsFree(base):
                createOrder(counter_id,0.5,DESTINATIONS_COORDS[dest],base)
                counter_id +=1
                state = "create_order"