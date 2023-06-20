import heapq
import math

class Graph:
    def __init__(self, vertices):
        self.vertices = vertices
        self.adjacency_list = [[] for _ in range(vertices)]

    def add_edge(self, u, v, w):
        self.adjacency_list[u].append((v, w))
        self.adjacency_list[v].append((u, w))

    def dijkstra(self, source):
        distance = [math.inf] * self.vertices
        distance[source] = 0
        pq = [(0, source)]
        while pq:
            (d, u) = heapq.heappop(pq)
            if d > distance[u]:
                continue
            for (v, w) in self.adjacency_list[u]:
                if distance[u] + w < distance[v]:
                    distance[v] = distance[u] + w
                    heapq.heappush(pq, (distance[v], v))
        return distance


class Robot:
    def __init__(self, start, goal, obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

    def collision_check(self, x, y):
        for (cx, cy, r) in self.obstacles:
            if (x - cx)**2 + (y - cy)**2 <= r**2:
                return True
        return False

    def plan(self):
        g = Graph(10000)
        for i in range(100):
            for j in range(100):
                if not self.collision_check(i, j):
                    if i < 99 and not self.collision_check(i+1, j):
                        g.add_edge(i*100+j, (i+1)*100+j, 1)
                    if j < 99 and not self.collision_check(i, j+1):
                        g.add_edge(i*100+j, i*100+j+1, 1)
        start_x, start_y = self.start
        goal_x, goal_y = self.goal
        source = start_x*100+start_y
        destination = goal_x*100+goal_y
        distance = g.dijkstra(source)
        path = []
        current = destination
        while current != source:
            path.append((current//100, current%100))
            for (v, _) in g.adjacency_list[current]:
                if distance[v] < distance[current]:
                    current = v
                    break
        path.append((source//100, source%100))
        path.reverse()
        return path

# Example usage
start = (0, 0)
goal = (99, 99)
obstacles = [(30, 30, 10), (70, 70, 10)]
robot = Robot(start, goal, obstacles)
path = robot.plan()
print(path)
