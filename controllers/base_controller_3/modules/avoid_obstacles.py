# Questo modulo serve per implementare i comportamenti in caso di ostacoli del robot
import math as m


# funzione logaritmo che calcola la sensibilità della soglia, proporzionale alla velocità
def function(x):
    return m.log(m.sqrt(x) + 1, 13) * 1.065


# funzione che ritorna il valore limite oltre il quale un sensore segnala un ostacolo
def limit(velocity, lim):
    sensibility = function(velocity)
    if sensibility * lim <= 1:
        return 1
    else:
        return sensibility * lim


# funzione che verifica se il valore di un sensore sonar supera il limite o meno.
def avoid_obstacles_sensor(value, velocity):
    if value is None or value >= limit(velocity, 1.99):
        return False
    else:
        return True


# funzione per la verifica di ostacoli e per il passaggio allo stato "avoid_obstacles"
def avoid_obstacles_full(front_left, front_right, upper_sensor, front_sensor, left_sensor, right_sensor, velocity):
    return avoid_obstacles_sensor(front_left, velocity[0]) or avoid_obstacles_sensor(front_right, velocity[
        0]) or avoid_obstacles_sensor(front_sensor, velocity[0]) or avoid_obstacles_sensor(upper_sensor, velocity[1]) or \
           avoid_obstacles_sensor(left_sensor, velocity[0]) or avoid_obstacles_sensor(right_sensor, velocity[0])
