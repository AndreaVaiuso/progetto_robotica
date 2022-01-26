# Questo modulo serve per implementare i comportamenti in caso di ostacoli del robot


def avoid_obstacles_left(value):
    pass


def avoid_obstacles_right(value):
    pass


def avoid_obstacles_upper(value):
    pass


def avoid_obstacles_lower(value):
    pass


def avoid_obstacles_front(value):
    pass


def avoid_obstacles_full(left_sensor, right_sensor, upper_sensor, lower_sensor, front_sensor):
    avoid_obstacles_left(left_sensor)
    avoid_obstacles_right(right_sensor)
    avoid_obstacles_front(front_sensor)
    avoid_obstacles_lower(lower_sensor)
    avoid_obstacles_upper(upper_sensor)
