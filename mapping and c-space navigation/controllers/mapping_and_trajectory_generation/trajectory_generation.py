import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from skimage.draw import line_nd
from collections import defaultdict, deque
from heapq import heapify, heappush, heappop

sns.set_theme(style='white', palette=None)
plt.ion()

class CspaceSample():
    def __init__(self, space):
        self.space = np.array(space)

        #self.fig, self.ax = plt.subplots()
        self.points_to_plot = []
        self.counter = 0

    def check_collision(self, q1, q2):
        x_points, y_points = line_nd(tuple(q1), tuple(q2), integer=True)

        for x, y in zip(x_points, y_points):
            if not (0 <= x < self.space.shape[0] and 0 <= y < self.space.shape[1]):
                return True, (x, y)
            if self.space[x][y] == False:
                return True, (x, y)

        return False, (x, y)
    
    def random_node(self, probability, qend):
        if np.random.rand()<probability:
            return np.array(qend)
        else:
            return np.array([np.random.randint(0, c-1) for c in self.space.shape])
    
    def nearest_node(self, graph, qstart, qrand):
        min_distance = float('inf')
        closest_point = None

        queue = [(np.linalg.norm(np.array(qstart) - qrand), qstart)]
        heapify(queue)
        seen = set()

        while queue:
            d, node = heappop(queue)
            if d < min_distance:
                min_distance = d
                closest_point = node
            
            for adj in graph[node]:
                if adj not in seen:
                    seen.add(adj)
                    heappush(queue, (np.linalg.norm(np.array(adj)-qrand), adj))
        if closest_point:
            return np.array(closest_point)
        else:
            return np.array(qstart)

    def rrt(self, qstart, qend, dq, num_nodes):
        graph = defaultdict(list)
        graph[qstart] = []

        # plt.imshow(self.space)
        # plt.ion()
        # self.ax.plot(qstart[1],qstart[0],'y*')
        # self.ax.plot(qend[1],qend[0],'y*')

        # debug
        print('RRT algorithm (internal): plotted given cspace')

        k = 0
        while k < num_nodes:

            qrand = self.random_node(0.2, qend)

            qnear = self.nearest_node(graph, qstart, qrand)

            vec = qrand - qnear
            norm = np.linalg.norm(vec)
            if norm == 0:
                continue
            norm_vec = vec / norm
            qnew = qnear + norm_vec*dq

            if np.linalg.norm(np.array(qend) - qnew) < dq:
                qnew = np.array(qend)

            is_collision, collision_point = self.check_collision(qnear, qnew)

            if not is_collision:
                # self.ax.plot([qnear[1],qnew[1]],[qnear[0],qnew[0]],'ro-', linewidth=1, markersize=2) 
                # plt.draw()
                # plt.pause(0.0001)
                
                graph[tuple(qnear)].append(tuple(qnew))
                k += 1

                if tuple(qnew) == qend:
                    print(f'Path found :) after {k} attempts')
                    return graph

            #else:
                #self.ax.plot([qnear[1],collision_point[1]],[qnear[0],collision_point[0]],'ko:', linewidth=1, markersize=1) 


        print(f'No path found :( after {k} attempts')
        return None

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
        if self.type == 'space':
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
                if self.type == 'space':
                    for p in path:
                        self.plot_path(p, 50, 'r')
                        plt.ioff()
                        plt.show()
                print(path)
                return path
            
            if self.type == 'space':
                adjacent_nodes = self.adjNode(u)
            elif self.type == 'graph':
                adjacent_nodes = self.graph[u]

            for adj in adjacent_nodes:
                if self.type == 'space':
                    rel_d, v = adj
                elif self.type == 'graph':
                    v = adj
                    rel_d = 1

                if self.type == 'space':
                    self.plot_path(v, 100)
                
                cur_d = d + rel_d 

                if cur_d < dist[v]:
                    dist[v] = cur_d
                    prev[v] = (u)
                    heuristic = np.sqrt((target[0] - v[0])**2 + (target[1] - v[1])**2)
                    heappush(q, (heuristic, cur_d, v))
        else:
            print('Target not reachable')
            if self.type == 'space':
                plt.ioff()
                plt.show()
            return None


if __name__ == '__main__':
    space = np.load('/Users/arthu/webots/mapping and c-space navigation/controllers/mapping_and_trajectory_generation/cspace.npy')

    cspace = CspaceSample(space)
    cspace = cspace.rrt((45, 10), (190, 230), 30, 200)

    graph = ShortestPath(cspace, type='graph')
    shortest_path = graph.dijkstra((45, 10), (190, 230))
