"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

# states list to mark completed states
states = ['DRIVE', 'ROTATE', 'TURN', 'STOP']

# set initial state
state = states[0] # DRIVE

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(0)
motor_right.setVelocity(0)

ds = [robot.getDevice(f'ps{i}') for i in range(8)]
for sensor in ds:
    sensor.enable(timestep)
    

while robot.step(timestep) != -1:
    
    # get dist sensor valeus
    dist_val = [sensor.getValue() for sensor in ds]
    print(f'dist values: {dist_val}')
    
    # state machine
    if state == states[0]: # DRIVE
        motor_left.setVelocity(3.14)
        motor_right.setVelocity(3.14)
        if dist_val[0] > 150 and dist_val[7] > 150:
            if states[1] == 'COMPLETED': # checks if 180deg rotation is completed
                state = states[2] 
            else:
                state = states[1]
                sensor_val = dist_val[7] - 10 # saves value of front left sensor
            
    if state == states[1]: # ROTATE
        motor_left.setVelocity(3.14)
        motor_right.setVelocity(-3.14)
        if dist_val[3] > sensor_val: # compares opposite sensor to check for rotation
            state = states[0]
            states[1] = 'COMPLETED' # to prevent two seperate DRIVE states
                   
    
    if state == states[2]: # TURN
        if dist_val[0] + dist_val[7] + dist_val[6] > 300:
            motor_left.setVelocity(3.14)
            motor_right.setVelocity(-3.14)
        else:
            motor_left.setVelocity(3.14)
            motor_right.setVelocity(3.14)
            if dist_val[5] < 80:
                state = states[3]

    if state == states[3]: # STOP
        motor_left.setVelocity(0)
        motor_right.setVelocity(0)
        break

# Enter here exit cleanup code.
