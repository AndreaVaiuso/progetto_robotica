import time
import struct
from controller import Supervisor, Emitter

BASE_COORDS = {"0":[0,0],"1":[2.17,3.18],"2":[-1,76,3.18]}


def createOrder(orderID,weight,position,base):
    message = struct.pack("ssssss","NO".encode('utf-8'),str(orderID).encode('utf-8'),str(weight).encode('utf-8'),str(position[0]).encode('utf-8'),str(position[1]).encode('utf-8'),str(base).encode('utf-8')) 
    print("Nuovo pacco>> ID:", orderID, "position: [",position[0],",",position[1],"]","base:",base)
    while emitter.send(message) != 1 :
        continue
    xp,yp = BASE_COORDS[str(base)]
    zp = 0.36
    root_node = supervisor.getRoot()
    children_field = root_node.getField("children")
    children_field.importMFNodeFromString(-1,"CardboardBox { translation "+str(yp)+" "+str(zp)+" "+str(xp)+"}")

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
emitter = supervisor.getDevice("emitter")
emitter.setChannel(Emitter.CHANNEL_BROADCAST)

createOrder(1,0.5,[10,10],1)
time.sleep(20)
createOrder(1,0.5,[20,-15],2)


while supervisor.step(timestep) != -1:
    pass

