import math


ricarica_a_quota = 10
prendere_ordine= 20
lasciare_ordine= 25
vel_media= 1.7 # per fare un metro impiega 1.7 sec
ricarica = 1 #in un secondo si ricarica di 100J
scarica = 2 #in due secondi si scarica di 200J
evita_ostacoli= 50 # non posso sapere se nel tragitto di consegna incontretà ostacoli e perderà tempo, devo aggiungere un punteggio che tiene conto in media del tempo che si può perdere
X_smistaggio=0 #dove si trova la base
Y_smistaggio=0 
def score_calculator(orders, current_order, pending_order, drone_pos, state, state_history, battery, recharge):
    if "drone_anomaly_detected" in state_history :
        score= 999999 # corrisponde ad un valore che non sarà mai superato, il drone non deve prendere più ordini in carico
        return score
    #pensare alle casistiche: prima mi occupo del current_order, poi ciclo su orders infine faccio le considerazioni su pending_order
    else:
        x_order=pending_order[4]
        y_order=pending_order[5]
        if current_order == [] and orders == [] and (state=="check_new_orders" or state=="check_battery" or state=="recharge_battery"): #caso base, qua considero solo pending_order, ricorda che tutti gli ordini nsono stati pending_order a loro volta e hanno influito cosi nel punteggio
            distanza_da_percorrere= math.sqrt(math.pow((drone_pos[0] - x_order), 2) + math.pow((drone_pos[1] - y_order), 2))*2
            t_p= distanza_da_percorrere*vel_media
            score = t_p+ricarica_a_quota+prendere_ordine+lasciare_ordine+evita_ostacoli
            return score
        elif current_order != [] and (state=="recharge_battery" or state=="check_battery"): #non può essere nello stato check_new_orders e avere current_order diverso dalla lista vuota
            t_r=(recharge-battery)/100 #secondi che occorrono per ricaricarsi
            distanza_da_percorrere_1= math.sqrt(math.pow((drone_pos[0] - current_order[4]), 2) + math.pow((drone_pos[1] - current_order[5]), 2))*2
            t_p_1= distanza_da_percorrere_1*vel_media
            pass #ricarica_a_quota, prendere_ordine, lasciare_ordine, vel_media; evita_ostacoli
        elif current_order != [] and state=="reach_quota":
            pass #ricarica_a_quota, prendere_ordine, lasciare_ordine, vel_media; evita_ostacoli
        elif current_order != [] and (state=="go_near_box" or state=="stabilize_on_position" or state=="lock_box"):
            pass #prendere_ordine, lasciare_ordine, vel_media; evita_ostacoli
        elif current_order != [] and ("reach_nav_altitude" in state_history):
            pass #lasciare_ordine, vel_media; evita_ostacoli
        elif current_order == [] and ("go_back_home" in state_history):  #se current_order è vuoto: 1) sto tornando alla base perche l'ho appena consegnato;  2)non ho ordini da esguire 
            pass #vel_media; evita_ostacoli
        return score
