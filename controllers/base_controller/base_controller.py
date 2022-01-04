"""base_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()
state = "check_new_orders"
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if state== "check_new_orders":
        #code
        pass
    elif state== "move_over_box":
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
    
   
  
