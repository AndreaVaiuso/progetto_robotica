from utils import euc_dist

def sccal(orders,pending,current,state_history,posit,base_coords):
    s_x = base_coords[0]
    s_y = base_coords[1]
    orders_new = orders.copy()
    orders_new.append(pending)
    tempo_percorso=0
    recharge_battery_to_reach_quote= 10
    reach_quote_to_lock_box= 30
    land_on_delivery_to_go_back_home= 30
    ostacoli= 50 # stiamo 50 secondi a schivare ostacoli in media durante un tragitto
    while(len(orders_new) > 0):
        order = orders_new.pop(0)
        if order[2] != -1:
            distanza= euc_dist([s_x, s_y], [order[4], order[5]])*2 #ogni metro impiego mediamente 1,7 s
        else:
            distanza_pacco= euc_dist([s_x,s_y], [order[6],order[7]]) # dalla base alla posizione del pacco
            distanza_consegna = euc_dist([order[6],order[7]], [order[4], order[5]])# dalla posizione del pacco alla consegna
            distanza_ritorno = euc_dist([s_x, s_y], [order[4], order[5]])# dalla consegna alla base
            distanza = distanza_pacco+distanza_consegna+distanza_ritorno
        tempo_percorso = tempo_percorso + distanza*1.7 + recharge_battery_to_reach_quote+reach_quote_to_lock_box+land_on_delivery_to_go_back_home+ostacoli
    
    if current==[]:
        return tempo_percorso
    else:
        if "go_back_home" in state_history:
            distanza_ritorno = euc_dist([s_x, s_y], [posit.x, posit.y])# da dove si trova il robot  alla base
            tempo_percorso= tempo_percorso+distanza_ritorno*1.7+(ostacoli/2)
        elif "land_on_delivery_station" in state_history:
            distanza_ritorno = euc_dist([s_x, s_y], [posit.x, posit.y])# da dove si trova il robot  alla base
            tempo_percorso= tempo_percorso+distanza_ritorno*1.7+land_on_delivery_to_go_back_home+(ostacoli/2)
        elif "reach_destination" in state_history:
            distanza_consegna = euc_dist([posit.x,posit.y], [current[4], current[5]])# dalla posizione del drone alla consegna
            distanza_ritorno = euc_dist([s_x, s_y], [current[4], current[5]])# dalla consegna alla base
            tempo_percorso= tempo_percorso+distanza_consegna*1.7+land_on_delivery_to_go_back_home+distanza_ritorno*1.7+(ostacoli/1.5)
        elif "reach_quote" in state_history:
            if current[2]!=-1:
                distanza= euc_dist([s_x, s_y], [current[4], current[5]])*2 #ogni metro impiego mediamente 1,7 s
                tempo_percorso = tempo_percorso + distanza*1.7 +reach_quote_to_lock_box+land_on_delivery_to_go_back_home+ostacoli
            else:
                distanza_pacco= euc_dist([s_x,s_y], [current[6],current[7]]) # dalla base alla posizione del pacco
                distanza_consegna = euc_dist([current[6],current[7]], [current[4], current[5]])# dalla posizione del pacco alla consegna
                distanza_ritorno = euc_dist([s_x, s_y], [current[4], current[5]])# dalla consegna alla base
                distanza = distanza_pacco+distanza_consegna+distanza_ritorno
                tempo_percorso = tempo_percorso + distanza*1.7 +reach_quote_to_lock_box+land_on_delivery_to_go_back_home+ostacoli
        elif "check_new_orders" in state_history:
            if current[2]!=-1:
                distanza= euc_dist([s_x, s_y], [current[4], current[5]])*2 #ogni metro impiego mediamente 1,7 s
                tempo_percorso = tempo_percorso + distanza*1.7+recharge_battery_to_reach_quote+reach_quote_to_lock_box+land_on_delivery_to_go_back_home+ostacoli
            else:
                distanza_pacco= euc_dist([s_x,s_y], [current[6],current[7]]) # dalla base alla posizione del pacco
                distanza_consegna = euc_dist([current[6],current[7]], [current[4], current[5]])# dalla posizione del pacco alla consegna
                distanza_ritorno = euc_dist([s_x, s_y], [current[4], current[5]])# dalla consegna alla base
                distanza = distanza_pacco+distanza_consegna+distanza_ritorno
                tempo_percorso = tempo_percorso + distanza*1.7+recharge_battery_to_reach_quote+reach_quote_to_lock_box+land_on_delivery_to_go_back_home+ostacoli
        return tempo_percorso