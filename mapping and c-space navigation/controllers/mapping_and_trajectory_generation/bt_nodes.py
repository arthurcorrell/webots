import numpy as np
import os
from scipy import signal
from os.path import exists

from controller import Supervisor

# constructors for behaviour tree-specific variables
from behaviour_tree import Context, BlackBoard, LeafNode

from trajectory_generation import CspaceSample, ShortestPath

# create blackboard for communication between nodes
blackboard = BlackBoard()

waypoints = [(-0.6, 0), (0.5, -0.2), (0.4, -3), (-1.65, -3), (-1.65, -1.5), (-1.6, -0), (-0.6, 0.5)]
blackboard.write('waypoints', waypoints)

# initialise device connections for behaviour tree in context object
robot = Supervisor()
context = Context()
context.initialise(timestep=int(robot.getBasicTimeStep()), max_speed = 10.1, 
                  left_motor = robot.getDevice('wheel_left_joint'), right_motor = robot.getDevice('wheel_right_joint'),
                  gps = robot.getDevice('gps'), compass = robot.getDevice('compass'), map = np.zeros((200, 250)), 
                  lidar = robot.getDevice('Hokuyo URG-04LX-UG01'), display = robot.getDevice('display'), 
                  marker = robot.getFromDef("marker").getField("translation"), blackboard=blackboard)



# subclass instances of behaviour tree nodes

class LoadCspace(LeafNode):
    def __init__(self, file, name=None):
        super().__init__()
        #super().__init__(context=context, blackboard=blackboard)
        self.file = file

    def initialise(self):
        global context, blackboard, robot
        self.status = 'RUNNING'
  
    def update(self):
        global context, blackboard, robot

        if exists(self.file):
            cspace = np.load(self.file)
            blackboard.write('cspace', cspace)
            self.status = 'SUCCESS'
        else:
            self.status = 'FAIL'
    
    def terminate(self, new_status):
        self.status = new_status

class CreateCspace(LeafNode):
    def __init__(self, file, name=None):
        super().__init__()
        #super().__init__(context=context, blackboard=blackboard)
        self.file = file

    def initialise(self):
        global context, blackboard, robot
        
        self.status = 'RUNNING'

        context.gps.enable(context.timestep)
        context.compass.enable(context.timestep)
        context.lidar.enable(context.timestep)
        context.lidar.enablePointCloud()
        robot.step(context.timestep)

    def update(self):
        global context, blackboard, robot
        # read and clean lidar data
        
        # res: 0.36deg = 667 parts
        # -> minus first and last 80 = 507
        # fov: 507 parts * 0.36 deg = 183 deg
        # -> 3.2 rad

        # get world frame orientation with compass data
        theta=np.arctan2(context.compass.getValues()[0],context.compass.getValues()[1])

        # get world frame coordinates with gps data
        xw = context.gps.getValues()[0]
        yw = context.gps.getValues()[1]
        
        ranges = np.array(context.lidar.getRangeImage()) 
        
        # clean data
        ranges = ranges[80:len(ranges)-80]
        angles = np.linspace(1.6, -1.6, 507)
        ranges[ranges == np.inf] = 100
        
        # displacement of lidar sensor in Tiago positive x axis
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
        context.display.setColor(0xFFFFFF)

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
            context.map[px, py] = min(context.map[px, py] + 0.01, 1.0)
        
            # intensity value and color
            v = int(context.map[px, py] * 255)
            color = (v << 16) + (v << 8) + v  # Convert grayscale to RGB
            context.display.setColor(color)
            context.display.drawPixel(px, py)
            
            context.display.setColor(0xFF0000)
            context.display.drawPixel(t1, t2)

    def terminate(self, new_status):
        self.status = new_status

        context.gps.disable()
        context.compass.disable()
        context.lidar.disable()
        context.lidar.disablePointCloud()

        # 2d configuration space 
        # 3rd degree of freedom ommitted due to radial symmetry along z-axis
                    
        # convolve map
        kernel = np.ones((20, 20))
        cmap = signal.convolve2d(context.map, kernel, mode='same')
                    
        # thresholding
        cspace = cmap > 0.9
        context.display.setColor(0x0000FF)

        # save map
        np.save(self.file, cspace)
        blackboard.write('cspace', cspace)
            
        for i in range(200):
            for j in range(250):
                if cmap[i, j]:
                    context.display.drawPixel(i, j) 

class FollowTrajectory(LeafNode):
    def __init__(self, name=None):
        super().__init__()
        #super().__init__(context=context, blackboard=blackboard)
        self.index = 1
        self.clockwise = True

    def initialise(self):
        global context, blackboard, robot
        
        self.status = 'RUNNING'
        self.tick = 0

        context.left_motor.setPosition(float('inf'))
        context.right_motor.setPosition(float('inf'))
        context.gps.enable(context.timestep)
        context.compass.enable(context.timestep)
        robot.step(context.timestep)

    def update(self):
        global context, blackboard, robot

        # read from blackboard 
        waypoints = blackboard.read('waypoints')

        # get world frame coordinates with gps data
        xw = context.gps.getValues()[0]
        yw = context.gps.getValues()[1]  

        # get world frame orientation with compass data
        theta=np.arctan2(context.compass.getValues()[0],context.compass.getValues()[1])

        context.marker.setSFVec3f([*waypoints[self.index], 1])

        # compute error terms
        rho = np.sqrt((xw-waypoints[self.index][0])**2+(yw-waypoints[self.index][1])**2)
        alpha = np.arctan2(waypoints[self.index][1]-yw, waypoints[self.index][0]-xw)-theta
        
        if alpha > 1.57:
            alpha -= 6.28
        if alpha < -1.57:
            alpha += 6.28
        
        # trajectory following
        if self.clockwise:
            if rho < 0.3 and self.index < len(waypoints) - 1:
                self.index += 1
                context.marker.setSFVec3f([*waypoints[self.index], 0])
            elif rho < 0.3 and self.index == len(waypoints) - 1:
                self.clockwise = False
        else:
            if rho < 0.3 and self.index > 0:
                self.index -= 1
                context.marker.setSFVec3f([*waypoints[self.index], 0])
            elif rho < 0.3 and self.index == 0:
                self.status = 'SUCCESS'  
                            
            if self.index == len(waypoints)-1:
                self.clockwise = False
            
        # reactive controller for trajectory following    
        
        # tune gain constants: feedback control
        p1 = 3.5
        p2 = 1.75
        
        # proportional control with gain constant
        dphil = -alpha * p1 + rho * p2
        dphir = alpha * p1 + rho * p2
        
        dphil = np.clip(dphil, -context.max_speed, context.max_speed)
        dphir = np.clip(dphir, -context.max_speed, context.max_speed)
        
        context.left_motor.setVelocity(dphil)
        context.right_motor.setVelocity(dphir)

        if self.index == 0 and not self.clockwise:
            blackboard.write('current position', (xw, yw))
            self.status = 'SUCCESS'

        self.tick += 1

    def terminate(self, new_status):
        self.status = new_status
        self.index = 0

        context.left_motor.setVelocity(0)
        context.right_motor.setVelocity(0)
        context.gps.disable()
        context.compass.disable()

class CreateTrajectory(LeafNode):
    def __init__(self, target, name=None):
        super().__init__()
        #super().__init__(context=context, blackboard=blackboard)
        self.target = target

    def initialise(self):
        global context, blackboard, robot
        
        self.status = 'RUNNING'
        context.gps.enable(context.timestep)
        robot.step(context.timestep)

    def update(self):
        global context, blackboard, robot

        # read current cspace and position
        if blackboard.read('current position'):
            start_world_point = blackboard.read('current position')
            start_xw, start_yw = start_world_point
        else:
            # get world frame coordinates with gps data
            start_xw = context.gps.getValues()[0]
            start_yw = context.gps.getValues()[1]
        # map world x to display x
        start_px = int(np.interp(start_xw,[-2.1, 2.1], [199, 0]))
        # map world y to display y
        start_py = int(np.interp(start_yw,[-3.7, 1.6], [0, 249]))
        start = (start_px, start_py)


        target_xw, target_yw = self.target
        # map world x to display x
        target_px = int(np.interp(target_xw,[-2.1, 2.1], [199, 0]))
        # map world y to display y
        target_py = int(np.interp(target_yw,[-3.7, 1.6], [0, 249]))
        target = (target_px, target_py)

        space = blackboard.read('cspace')

        space = np.load('/Users/arthu/webots/mapping and c-space navigation/controllers/mapping_and_trajectory_generation/cspace.npy')


        # debug 1
        print('BLACKBOARD: read cspace')
        print(space)


        cspace = CspaceSample(space)

        # debug 1.5
        print('RRT algorithm: initialized cspace object')

        sampled_cspace = cspace.rrt(start, target, 10, 300)

        # debug 2
        print('RRT algorithm: sampled cspace')
        print(sampled_cspace)

        graph = ShortestPath(sampled_cspace, type='graph')
        map_shortest_path = graph.dijkstra(start, target)

        # debug 3
        print(f'Dijkstra algorithm: found path from {start_world_point} to {self.target}')
    

        shortest_path = []
        for path_px, path_py in map_shortest_path:
            # map display x to world x
            path_xw = int(np.interp(path_px, [199, 0], [-2.1, 2.1]))
            # map display y to world y
            path_yw = int(np.interp(path_py, [0, 249], [-3.7, 1.6]))
            path_point = (path_xw, path_yw)
            shortest_path.append(path_point)

        if shortest_path != None:
            blackboard.write('waypoints', shortest_path)
            self.status = 'SUCCESS'
        else:
            self.status = 'FAIL'

    def terminate(self, new_status):
        self.status = new_status