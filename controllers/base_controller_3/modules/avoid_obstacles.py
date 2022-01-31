# Questo modulo serve per implementare i comportamenti in caso di ostacoli del robot


def avoid_obstacles_left(value):
    if value is None or value >= 1.5:
        return False
    else:
        return True


def avoid_obstacles_right(value):
    if value is None or value >= 1.5:
        return False
    else:
        return True


def avoid_obstacles_upper(value):
    if value is None or value >= 1.5:
        return False
    else:
        return True


def avoid_obstacles_lower(value):
    if value is None or value >= 1.5:
        return False
    else:
        return True


def avoid_obstacles_front(value):
    if value is None or value >= 1.5:
        return False
    else:
        return True


def avoid_obstacles_full(left_sensor, right_sensor, upper_sensor, lower_sensor, front_sensor):
    return avoid_obstacles_left(left_sensor) or avoid_obstacles_right(right_sensor) or avoid_obstacles_front(
        front_sensor) or avoid_obstacles_front(front_sensor) or avoid_obstacles_lower(
        lower_sensor) or avoid_obstacles_upper(upper_sensor)
