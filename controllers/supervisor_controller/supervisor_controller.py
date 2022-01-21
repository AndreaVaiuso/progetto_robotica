import time
import struct
import threading
from controller import Supervisor, Emitter

BASE_COORDS = {"0":[0,0],"1":[2.17,3.18],"2":[-1.76,3.18]}

def createOrder(orderID,weight,position,base):
    message = struct.pack("ciiddd",b"N",int(orderID),int(base),float(weight),float(position[0]),float(position[1]))
    # message = struct.pack("cifffi","N",int(orderID),float(weight),float(position[0],float(position[1]),int(base)))
    # message = struct.pack("ssssss","NO".encode('utf-8'),str(orderID).encode('utf-8'),str(weight).encode('utf-8'),str(position[0]).encode('utf-8'),str(position[1]).encode('utf-8'),str(base).encode('utf-8')) 
    print("Nuovo pacco>> ID:", orderID, "position: [",position[0],",",position[1],"]","base:",base)
    while emitter.send(message) != 1 :
        continue
    xp = BASE_COORDS[str(base)][0]
    yp = BASE_COORDS[str(base)][1]

    zp = 0.26
    root_node = supervisor.getRoot()
    children_field = root_node.getField("children")
    children_field.importMFNode(-1,"../../protos/box.wbo") 
    node = children_field.getMFNode(-1) 
    field = node.getField("translation") 
    field.setSFVec3f([yp,zp,xp])


def order():
    createOrder(1,0.5,[10,10],1)

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
emitter = supervisor.getDevice("emitter")
emitter.setChannel(Emitter.CHANNEL_BROADCAST)

th = threading.Thread(target=order,args=[])
th.start()


while supervisor.step(timestep) != -1:
    break
       

