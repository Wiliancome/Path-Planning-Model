解释一下代码：

import heapq
import math
这个代码段导入了 Python 的堆队列操作库 heapq 和数学运算库 math。

class Graph:
    def __init__(self, vertices):
        self.vertices = vertices
        self.adjacency_list = [[] for _ in range(vertices)]
这段代码定义了 Graph 类，该类用于表示机器人在网格空间内的运动。类 __init__ 方法当创建 Graph 类的对象时，执行此方法，初始化顶点数和邻接列表。

    def add_edge(self, u, v, w):
        self.adjacency_list[u].append((v, w))
        self.adjacency_list[v].append((u, w))
这段代码定义了用于添加边到邻接列表的 add_edge 方法，并按照无向图的方式将边添加到邻接列表中。

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
这段代码定义了 dijkstra 方法，该方法执行 Dijkstra 算法来计算机器人的最短路径。此方法首先初始化从源顶点到所有其它顶点的距离，然后按照距离从小到大的顺序将每个顶点压入堆中，接着按照无向图的方式遍历邻接列表，并更新所有与当前顶点相邻的顶点的最小距离，最后返回最短距离列表。（具体 Dijkstra 算法的原理可以参考相关资料）

class Robot:
    def __init__(self, start, goal, obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
这段代码定义了 Robot 类，该类代表移动机器人的属性和行为。在此定义机器人的起点、目标点和障碍物信息。

    def collision_check(self, x, y):
        for (cx, cy, r) in self.obstacles:
            if (x - cx)**2 + (y - cy)**2 <= r**2:
                return True
        return False
这段代码定义了一个碰撞检测函数 collision_check，该函数判断机器人当前位置是否与任何障碍物相撞。针对机器人当前位置及障碍物的位置信息进行判断，如果检测到碰撞则返回 True，否则返回 False。

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
这段代码定义了 plan 方法，该方法利用 Graph 类和 dijkstra 方法来规划机器人的路径plan 方法用 Graph 类构建了一个网格图，然后遍历网格图的每个顶点来寻找机器人行走的合法路径。针对每个非障碍物位置，确定其向右和向下的邻居位置是否空闲，若空闲则添加一个连接相邻点的边到邻接列表中。

接下来，plan 方法定义起点和终点所在的位置，并使用 Dijkstra 算法计算出最短路径。为了得到一条真正的路径，需要追溯寻找从起点到终点的路径，这需要遍历邻接列表，寻找与当前顶点有连通边的顶点，编号最小的顶点即为最近的位置，最后返回机器人的路径。

# Example usage
start = (0, 0)
goal = (99, 99)
obstacles = [(30, 30, 10), (70, 70, 10)]
robot = Robot(start, goal, obstacles)
path = robot.plan()
print(path)
这段代码示例为机器人规划运动轨迹，定义起点和终点，同时给定障碍。最后通过调用 robot 对象的 plan() 方法来计算机器人的最佳路径，并将结果打印出来。
