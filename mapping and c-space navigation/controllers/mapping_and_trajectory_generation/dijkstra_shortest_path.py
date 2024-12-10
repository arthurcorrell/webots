from collections import defaultdict
from heapq import heapify, heappush, heappop
import numpy as np
import matplotlib.pyplot as plt

class ShortestPath():
    def __init__(self, graph, type='graph'):
        valid_types = {'graph', 'space'}
        if type not in valid_types:
            raise ValueError(f"Invalid type {type}. Only accepts 'graph' and 'space'")
        self.type = type
        self.graph = graph

        self.points_to_plot = []
        self.counter = 0
            
    def plot_path(self, node, cooldown, color='g'):
        self.points_to_plot.append((node[1], node[0])) 
        self.counter += 1
        try:
            if self.counter % cooldown == 0:
                for point in self.points_to_plot:
                    plt.plot(*point, f'{color}*')  
                plt.draw()
                plt.pause(0.0001)
                self.points_to_plot = [] 
        except KeyboardInterrupt:
            plt.close('all')

    def adjNode(self, u):
        i, j = u[0], u[1]
        adj = []
        if (0 <= i < len(self.graph)) and (0 <= j < len(self.graph[0])):
            for delta in (1, 0), (-1, 0), (0, 1), (0, -1), (-1, -1), (-1, 1), (1, -1), (1, 1):
                x, y = i + delta[0], j + delta[1]
                dist = np.sqrt(delta[0]**2+delta[1]**2)
                if (0 <= x < len(self.graph)) and (0 <= y < len(self.graph[0])):
                    if self.graph[x][y]:
                        adj.append((dist,(x, y)))
        return adj

    def dijkstra(self, start, target):

        plt.imshow(self.graph)
        plt.ion()
        plt.plot(target[1],target[0],'y*')
        
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
                    self.plot_path(p, 50, 'r')

                plt.ioff()
                plt.show()

                return path
            
            if self.type == 'space':
                adjacent_nodes = self.adjNode(u)
            elif self.type == 'graph':
                pass

            for adj in adjacent_nodes:
                rel_d, v = adj

                self.plot_path(v, 100)
                
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


if __name__=='__main__':
    space = np.load('/Users/arthu/webots/mapping and c-space navigation/controllers/mapping_and_trajectory_generation/cspace.npy')

    cspace = ShortestPath(space, type='space')
    shortest_path = cspace.dijkstra((45, 10), (190, 230))
