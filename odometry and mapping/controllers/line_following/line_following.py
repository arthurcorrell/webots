"""line_following controller."""

from controller import Robot, Lidar
import numpy as np
from matplotlib import pyplot as plt


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
max_speed = 6.28

left_motor, right_motor = robot.getDevice('left wheel motor'), robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')
fig, ax = plt.subplots()
plt.ion()
robot.step(timestep)

gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for sensor in gs:
    sensor.enable(timestep)
    
    
# e-pucks position in world coordinate system
alpha = 1.57 # e-puck angle relative to world axis
xw = 0 # e-pucks translation on the world x axis
yw = 0.028 # e-pucks translation on world y axis

# odometer function
dt = timestep / 1000
r = 0.0201
d = 0.052
time = 0 
dist = 0.028
angle = 90

# Main loop:
# wheel speeds set once, pureley reactive (no states)
while robot.step(timestep) != -1:
    # Read the sensors:

    g = [sensor.getValue() for sensor in gs]
    
    # loop closure by stopping condition
    if g[0] + g[1] + g[2] < 910 and alpha < -4.5: # -4.5 is abit less than 3/4 rotations

            # error assumes final e-puck position is (0, 0, 0)
        error = np.sqrt(xw**2+yw**2) 
        print(f'Stopping condition found! Error: {round(error, 2)}m')
        
        break

    if g[0] > 600 and g[1] < 400 and g[2] > 600:
        dphir = dphil = max_speed
        
    elif g[2] < 600:
        dphil, dphir = max_speed * 0.3, 0 * max_speed
        
    elif g[0] < 600:
        dphir, dphil = max_speed * 0.3, 0.1 * max_speed
        
    left_motor.setVelocity(dphil)
    right_motor.setVelocity(dphir)
        
    # odometer
    
    # incremental displacement dx (not change in x coordinate) for timestep dt:
    dx = ((r*dphir + r*dphil)/2) * dt 
    # incremental rotation relative to world frame for timestep dt:
    dw = ((r*dphir - r*dphil)/d) * dt
    
    time += 0.032 # total time elapsed
    dist += dx # total displacement (-> length of line)
    angle += (dw/3.14) + 180 # total rotation in degrees
    
    # localization
    # if alpha=0 or 6.28, e-puck is translated incremental displacement dx for  
        # incremental timestep dt only on the world frame x-axis
    xw += np.cos(alpha) * dx 
    # if alpha=3.14, e-puck is translated (...) dx for (...) dt only on world frame y-axis
        # ...and so on 
    yw += np.sin(alpha) * dx
    
    alpha += dw
   
    print(f'World Coordinates: X: {round(xw, 3)}, Y: {round(yw, 3)}, Angle: {round(alpha, 3)}')
    
    
    # read lidar data
    ranges = np.array(lidar.getRangeImage())
    angles = np.linspace(3.14, -3.14, 360)
    
    # clean data
    ranges[ranges == np.inf] = 100
    
    # coords in robot frame
    p_r = np.array([np.cos(angles)*ranges, np.sin(angles)*ranges, np.ones(360)])
    
    # homogenous transform
    w_T_r = np.array([[np.cos(alpha), -np.sin(alpha), xw],
                      [np.sin(alpha),  np.cos(alpha), yw],
                      [0,              0,              1]])
    
    # coords in world frame
    w_r = w_T_r @ p_r
    
    plt.plot(w_r[0, :], w_r[1, :], '.')
    plt.pause(0.01)
    plt.show()
    
        
    pass

# Enter here exit cleanup code.
left_motor.setVelocity(0)
right_motor.setVelocity(0)