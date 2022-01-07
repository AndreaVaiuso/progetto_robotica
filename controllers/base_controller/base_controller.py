"""base_controller controller."""
from controller import Robot,Receiver, Emitter 
import threading
import random
import struct
import modules.communication as comm
import time
import operator

orders = []
battery = 100
BASE_COORDS = {"0":[1,2,3]}


# SUPERVISOR_EMITTER_CHANNEL = 0 non capisco il supervisor deve emettere in broadcast, ogni robot ha un suo canale di ricezione



robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)
name = robot.get_name()
canale_di_ricezione =int(name[-1])
receiver.setChannel(canale_di_ricezione)
state = "check_new_orders"
current_order =''
score_dict={}

def score_calculator(dataList):
    return canale_di_ricezione

def send_score(dataList):
    score = score_calculator(dataList)
    # fare la funzione 
    #ci serve anche ORDER_ID per capire quello score a cosa si riferisce
    message = struct.pack("chd",'S', name, dataList[1],  score) #dataList è una tupla
    emitter.setChannel(CHANNEL_BROADCAST) #dovrebbe inviare su tutti i canali cosi
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
    
   
  
