# Questo modulo serve per implementare i comportamenti in caso di ostacoli del robot
import math as m


def function(x):
    return m.log(m.sqrt(x) + 1, 13) * 1.065


def limit(velocity, lim):
    return function(velocity) * lim


def avoid_obstacles_sensor(value, velocity):
    if value is None or value >= limit(velocity, 1.99):
        return False
    else:
        return True


def avoid_obstacles_full(upper_sensor, front_sensor, velocity):
    return avoid_obstacles_sensor(front_sensor, velocity[0]) or avoid_obstacles_sensor(upper_sensor, velocity[1])
