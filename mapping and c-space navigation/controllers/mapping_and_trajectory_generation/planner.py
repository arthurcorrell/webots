from collections import defaultdict
from heapq import heapify, heappush, heappop
import numpy as np
import matplotlib.pyplot as plt

class Planner():
    def __init__(self, map):
        self.map = map

    def adjNode(self, u):
        i, j = u[0], u[1]
        adj = []
        if (0 <= i < len(self.map)) and (0 <= j < len(self.map[0])):
            for delta in (1, 0), (-1, 0), (0, 1), (0, -1), (-1, -1), (-1, 1), (1, -1), (1, 1):
                x, y = i + delta[0], j + delta[1]
                dist = np.sqrt(delta[0]**2+delta[1]**2)
                if (0 <= x < len(self.map)) and (0 <= y < len(self.map[0])):
                    if not self.map[x][y]:
                        adj.append((dist,(x, y)))
        return adj

    def dijkstra(self, start, target):

        plt.imshow(self.map)
        plt.ion()
        plt.plot(target[1],target[0],'y*')

        points_to_plot = []
        counter = 0
        cooldown = 10
        
        visited = set()
        dist = defaultdict(lambda: float('inf'))
        dist[start] = 0
        prev = defaultdict(lambda: None)

        heuristic = np.sqrt((target[0]-start[0])**2+(target[1]-start[1])**2)
        
        q = [(heuristic, dist[start], start)]
        heapify(q)

        while q:
            _, d, u = heappop(q)

            if u in visited:
                continue
            visited.add(u)

            if u == target:
                path = []
                key = u
                while key:
                    path.append(key)
                    key = prev[key]

                for p in path:    
                    plt.plot(p[1],p[0],'r.')
                    plt.draw()
                    plt.pause(0.0001)
                plt.ioff()
                plt.show()

                return path

            for adj in self.adjNode(u):
                rel_d, v = adj

                points_to_plot.append((v[1], v[0]))  # Append points to batch
                counter += 1
                if counter % cooldown == 0:  # Update at intervals
                    for point in points_to_plot:
                        plt.plot(*point, 'g*')  # Batch plot
                    plt.draw()
                    plt.pause(0.0001)
                    points_to_plot = []  # Clear batch
                
                cur_d = d + rel_d 

                if cur_d < dist[v]:
                    dist[v] = cur_d
                    prev[v] = (u)
                    heuristic = np.sqrt((target[0] - v[0])**2 + (target[1] - v[1])**2)
                    heappush(q, (heuristic, cur_d, v))
        else:
            print('Target not reachable')
            plt.ioff()
            plt.show()
            return None



map = np.load('/Users/arthu/webots/mapping and c-space navigation/controllers/mapping_and_trajectory_generation/cspace.npy')

try:

    Planner(map).dijkstra((0, 0), (19, 5))

except KeyboardInterrupt:
    plt.close('all')