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


# SUPERVISOR_EMITTER_CHANNEL = 0 non capisco il supervisor deve emettere in broadcast, ogni robot ha un suo canale di ricezione



robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)
nome = robot.get_name()
canale_di_ricezione =int(nome[(len(nome)-1])
receiver.setChannel(canale_di_ricezione)
state = "check_new_orders"

def score_calculator(datalist):
    pass 

def update_orders():
    global battery, orders
    while True:
        if receiver.getQueueLength() > 0: #ma dobbiamo distinguere fra due tipi di dati in arrivo , i nuovi ordini, e i punteggi
            x = receiver.getData()
            # [ "chd" , TYPE, ORDER_ID , WEIGHT , DESTINATION , BASE ]
            dataList = struct.unpack("chd",x) #chd è il formato di compressione
            receiver.nextPacket()
        score = score_calculator(dataList) # fare la funzione 
        #ci serve anche ORDER_ID per capire quello score a cosa si riferisce
        message = struct.pack("chd", DRONE_ID, dataList[1],  score) #dataList è una tupla
        emitter.setChannel(CHANNEL_BROADCAST) #dovrebbe inviare su tutti i canali cosi
        while emitter.send(message) != 1 : #se restituisce 0 significa che la coda di invio era piena e lo deve rifare
          print()
        



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
    
   
  
