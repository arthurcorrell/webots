
from controller import Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance -  can control environment
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
max_speed = 10.1

# get motors
left_motor, right_motor = robot.getDevice('wheel_left_joint'), robot.getDevice('wheel_right_joint')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# lidar and display for mapping
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')

lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')
fig, ax = plt.subplots()
plt.ion()
robot.step(timestep)

# odometry replacement
gps = robot.getDevice('gps')
gps.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)


# init probabilistic map at display resolution
map = np.zeros((200, 250))
    
# trajectory following with orange
marker_node = robot.getFromDef("marker")
clockwise = True
marker = marker_node.getField("translation")

# init waypoints clockwise from sink
wp = [(-0.6, 0), (0.5, -0.2), (0.4, -3), (-1.65, -3), (-1.65, -1.5), (-1.6, -0), (-0.6, 0.5)]
index = 1
# marker has no physics and floats to avoid lidar
marker.setSFVec3f([*wp[index], 0.5])

while robot.step(timestep) != -1:

    # get world frame coordinates with gps data
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    
    # get world frame orientation with compass data
    theta=np.arctan2(compass.getValues()[0],compass.getValues()[1])

    # compute error terms
    rho = np.sqrt((xw-wp[index][0])**2+(yw-wp[index][1])**2)
    alpha = np.arctan2(wp[index][1]-yw, wp[index][0]-xw)-theta
    
    if alpha > 1.57:
        alpha -= 6.28
    if alpha < -1.57:
        alpha += 6.28
    print(f'Error: {rho:.2f}, {alpha:.1f}')
    
    # trajectory following
    if clockwise:
        if rho < 0.3 and index < len(wp) - 1:
            index += 1
            marker.setSFVec3f([*wp[index], 0])
        elif rho < 0.3 and index == len(wp) - 1:
            clockwise = False
    else:
        if rho < 0.3 and index > 0:
            index -= 1
            marker.setSFVec3f([*wp[index], 0])
        elif rho < 0.3 and index == 0:
            break  
                        
        if index == len(wp)-1:
            clockwise = False
        
    # reactive controller for trajectory following    
    
    # tune gain constants: feedback control
    p1 = 3.5
    p2 = 1.75
    
    # proportional control with gain constant
    dphil = -alpha * p1 + rho * p2
    dphir = alpha * p1 + rho * p2
    
    
    dphil = np.clip(dphil, -max_speed, max_speed)
    dphir = np.clip(dphir, -max_speed, max_speed)


    
    left_motor.setVelocity(dphil)
    right_motor.setVelocity(dphir)

    if index == 0 and not clockwise:
        print("Simulation complete.")
        break
    
    
    # read and clean lidar data
    
    # res: 0.36deg = 667 parts
    # -> minus first and last 80 = 507
    # fov: 507 parts * 0.36 deg = 183 deg
    # -> 3.2 rad
    
    ranges = np.array(lidar.getRangeImage()) 
    
    # clean data
    ranges = ranges[80:len(ranges)-80]
    angles = np.linspace(1.6, -1.6, 507)
    ranges[ranges == np.inf] = 100
    
    # displacement of lidar sensor in Tiago pos-x axis
    displacement = 0.202
    
    # coords in robot frame
    p_r = np.array([np.cos(angles)*ranges+displacement, np.sin(angles)*ranges, np.ones(507)])
    
    # homogenous transform
    w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                      [np.sin(theta),  np.cos(theta), yw],
                      [0,              0,              1]])
    
    # coords in world frame R^3*360
    w_r = w_T_r @ p_r

    # visualization
    display.setColor(0xFFFFFF)

    # arena corners:
    #top right: 2.3, -3.9
    #bot right: -2.3, -3.9
    
    #top left: 2.3, 1.8
    #bot left: -2.3, 1.8
    
    # width: 5.7 - 250 pixels
    # height: 4.6 - 200 pixels
 

    for i in range(360):
        # map world coordinates to display
        # map world x to display x
        px = int(np.interp(w_r[0][i],[-2.1, 2.1], [199, 0]))
        # map world y to display y
        py = int(np.interp(w_r[1][i],[-3.7, 1.6], [0, 249]))
        
        t1 = int(np.interp(xw, [-2.1, 2.1], [199, 0]))
        t2 = int(np.interp(yw, [-3.7, 1.6], [0, 249]))        
        
        # probabilistic map
        # -> map[x, y] corresponds to display dimension
        map[px, py] = min(map[px, py] + 0.01, 1.0)
    
        # intensity value and color
        v = int(map[px, py] * 255)
        color = (v << 16) + (v << 8) + v  # Convert grayscale to RGB
        display.setColor(color)
        display.drawPixel(px, py)
        
        display.setColor(0xFF0000)
        display.drawPixel(t1, t2)

          
    
# 2d configuration space 
# 3rd degree of freedom ommitted due to radial symmetry along z-axis
            
# convolve map
kernel = np.ones((20, 20))
cmap = signal.convolve2d(map, kernel, mode='same')
            
# thresholding
cspace = cmap > 0.9
display.setColor(0x0000FF)

# save map
np.save('cspace', cspace)
    
for i in range(200):
    for j in range(250):
        if cmap[i, j]:
            display.drawPixel(i, j) 

      
# Enter here exit cleanup code.
left_motor.setVelocity(0)
right_motor.setVelocity(0)
