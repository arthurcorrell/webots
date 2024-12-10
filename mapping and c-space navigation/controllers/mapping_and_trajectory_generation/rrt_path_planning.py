import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from collections import defaultdict, deque
from heapq import heapify, heappush, heappop

sns.set_theme(style='white', palette=None)

class PathPlanner():
    def __init__(self, space):
        self.space = np.array(space)

        self.fig, self.ax = plt.subplots()
        self.points_to_plot = []
        self.counter = 0

    def bresenham_line(self, x1, y1, x2, y2):
        # generate points on a line between (x1, y1) and (x2, y2) with bresenham line algorithm
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        i = 0
        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
            i += 1
        return points

    def check_collision(self, q1, q2):

        x1, y1 = int(q1[0]), int(q1[1])
        x2, y2 = int(q2[0]), int(q2[1])
        line_points = self.bresenham_line(x1, y1, x2, y2)
        
        for x, y in line_points:
            if not (0 <= x < self.space.shape[0] and 0 <= y < self.space.shape[1]):
                return True, (x, y)
            if self.space[x][y] == False:
                return True, (x, y)

        return False, (x, y)
    
    def random_node(self, probability, qend):
        rand = np.random.randint(0, 1/probability)
        if rand == 1:
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
        
        return np.array(closest_point)

    def rrt(self, qstart, qend, dq, num_nodes):
        graph = defaultdict(list)
        graph[qstart] = []

        plt.imshow(self.space)
        plt.ion()
        self.ax.plot(qstart[1],qstart[0],'y*')
        self.ax.plot(qend[1],qend[0],'y*')

        k = 0
        while k < num_nodes:

            qrand = self.random_node(0.2, qend)

            qnear = self.nearest_node(graph, qstart, qrand)

            vec = qrand - qnear
            norm_vec = vec / np.linalg.norm(vec)
            qnew = qnear + norm_vec*dq

            if np.linalg.norm(np.array(qend) - qnew) < dq:
                qnew = np.array(qend)

            is_collision, collision_point = self.check_collision(qnear, qnew)

            if not is_collision:
                self.ax.plot([qnear[1],qnew[1]],[qnear[0],qnew[0]],'ro-', linewidth=1, markersize=2) 
                plt.draw()
                plt.pause(0.0001)
                
                graph[tuple(qnear)].append(tuple(qnew))
                k += 1

                if tuple(qnew) == qend:
                    print(f'Path found :) after {k} attempts')
                    return graph

            else:
                self.ax.plot([qnear[1],collision_point[1]],[qnear[0],collision_point[0]],'ko-', linestyle=':', linewidth=1, markersize=1) 

        plt.ioff()

        print(f'No path found :( after {k} attempts')
        return None


if __name__ == '__main__':
    space = np.load('/Users/arthu/webots/mapping and c-space navigation/controllers/mapping_and_trajectory_generation/cspace.npy')

    cspace = PathPlanner(space)
    graph = cspace.rrt((45, 10), (190, 230), 30, 200)

    plt.show()