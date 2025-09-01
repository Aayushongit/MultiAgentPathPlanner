import numpy as np
from collections import defaultdict, deque
import heapq
import sys
from task_parser import parseTask

def Merge(dict1, dict2):
    res = {**dict1, **dict2}
    return res
    
try:
    grid_param, blocked, roboSrc, tasks = parseTask(sys.argv[1])
except:
    grid_param, blocked, roboSrc, tasks = parseTask()

nodeType = [0 for i in range(len(roboSrc))] + [1 for i in range(len(tasks))]
nodeData = roboSrc + tasks
INT_MIN = float('-inf')
INT_MAX = float('inf')

class Conflict:
    def __init__(self, agent1, agent2, pos, time):
        self.agent1 = agent1
        self.agent2 = agent2
        self.pos = pos
        self.time = time

class Constraint:
    def __init__(self, agent, pos, time):
        self.agent = agent
        self.pos = pos
        self.time = time

class CBSNode:
    def __init__(self):
        self.constraints = []
        self.paths = {}
        self.cost = 0

class CBSRobo:

    def __init__(self, blocked, roboSrc, tasks, nodeData, nodeType, grid_param):
        self.x, self.y = grid_param
        self.blocked = blocked
        self.dist = 0
        self.graph = defaultdict(list)
        self.task = []
        self.tasks = tasks
        self.perms = []
        self.nodeData = nodeData
        self.nodeType = nodeType
        self.h, self.w = len(roboSrc), len(tasks)
        self.relMat = [[0 for j in range(self.h+self.w)] for i in range(self.h+self.w)]
        self.spanMin = INT_MAX

    def getPos(self):
        return (self.x, self.y)

    def manhattanDistance(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def getNeighbors(self, pos):
        neighbors = []
        x, y = pos
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            new_pos = (new_x, new_y)
            
            if (0 <= new_x < self.x and 0 <= new_y < self.y and 
                new_pos not in self.blocked):
                neighbors.append(new_pos)
        
        return neighbors

    def astarWithConstraints(self, start, goal, constraints):
        open_set = []
        heapq.heappush(open_set, (0, start, 0, [start]))
        
        visited = set()
        
        while open_set:
            f, current, time, path = heapq.heappop(open_set)
            
            if (current, time) in visited:
                continue
            visited.add((current, time))
            
            if current == goal:
                return path, len(path) - 1
            
            violates_constraint = False
            for constraint in constraints:
                if constraint.pos == current and constraint.time == time:
                    violates_constraint = True
                    break
            
            if violates_constraint:
                continue
            
            for neighbor in self.getNeighbors(current):
                if (neighbor, time + 1) not in visited:
                    new_path = path + [neighbor]
                    g = time + 1
                    h = self.manhattanDistance(neighbor, goal)
                    f = g + h
                    heapq.heappush(open_set, (f, neighbor, time + 1, new_path))
        
        return None, -1

    def findConflicts(self, paths):
        conflicts = []
        agents = list(paths.keys())
        
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                agent1, agent2 = agents[i], agents[j]
                path1, path2 = paths[agent1], paths[agent2]
                
                max_len = max(len(path1), len(path2))
                
                for t in range(max_len):
                    pos1 = path1[min(t, len(path1) - 1)]
                    pos2 = path2[min(t, len(path2) - 1)]
                    
                    if pos1 == pos2:
                        conflicts.append(Conflict(agent1, agent2, pos1, t))
        
        return conflicts

    def cbsSearch(self, starts, goals):
        root = CBSNode()
        
        for i, (start, goal) in enumerate(zip(starts, goals)):
            path, cost = self.astarWithConstraints(start, goal, [])
            if path is None:
                return None
            root.paths[i] = path
            root.cost += cost
        
        open_set = []
        heapq.heappush(open_set, (root.cost, id(root), root))
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            conflicts = self.findConflicts(current.paths)
            
            if not conflicts:
                return current.paths
            
            conflict = conflicts[0]
            
            for agent in [conflict.agent1, conflict.agent2]:
                new_node = CBSNode()
                new_node.constraints = current.constraints + [Constraint(agent, conflict.pos, conflict.time)]
                new_node.paths = current.paths.copy()
                
                start = new_node.paths[agent][0]
                goal = new_node.paths[agent][-1]
                
                path, cost = self.astarWithConstraints(start, goal, new_node.constraints)
                if path is not None:
                    new_node.paths[agent] = path
                    new_node.cost = sum(len(p) - 1 for p in new_node.paths.values())
                    heapq.heappush(open_set, (new_node.cost, id(new_node), new_node))
        
        return None

    def distMinAtoB(self, src, dst):
        paths = self.cbsSearch([src], [dst])
        if paths is None:
            print('no possible path from '+str(src)+" to "+str(dst))
            return -1
        
        return len(paths[0]) - 1

    def Search(self):
        for i in range(self.w+self.h):
            for j in range(self.w+self.h):
                robot, task = nodeType[i], nodeType[j]
                if robot == 0 and task == 1:
                    self.relMat[i][j] = self.distMinAtoB(self.nodeData[i][0], self.nodeData[j][0])
                elif robot == 1 and task == 1:
                    dist1 = self.distMinAtoB(self.nodeData[i][0], self.nodeData[i][1])
                    dist2 = self.distMinAtoB(self.nodeData[i][1], self.nodeData[j][0])
                    if dist1 == -1 or dist2 == -1:
                        self.relMat[i][j] = -1
                    else:
                        self.relMat[i][j] = dist1 + dist2
                elif robot == 1 and task == 0:
                    dist1 = self.distMinAtoB(self.nodeData[i][0], self.nodeData[i][1])
                    dist2 = self.distMinAtoB(self.nodeData[i][1], self.nodeData[j][1])
                    if dist1 == -1 or dist2 == -1:
                        self.relMat[i][j] = -1
                    else:
                        self.relMat[i][j] = dist1 + dist2
                else:
                    pass

    def allPossiblePaths(self, a, size, perms):
        if size == 1:
            perms.append([0] + a)
            return
        for i in range(size):
            self.allPossiblePaths(a, size-1, perms)
            
            if size & 1:
                a[0], a[size-1] = a[size-1], a[0]
            else:
                a[i], a[size-1] = a[size-1], a[i]

    def HamCycle(self):
        perms = []
        self.allPossiblePaths(list(range(1, self.h+self.w, 1)), self.w + self.h - 1, perms)
        return perms

    def getMakespan(self, pattern):
        a = 0
        for i in range(len(pattern)):
            if self.nodeType[pattern[i]] == 0:
                a = i
                break

        minm = INT_MIN
        pattern = pattern[a:] + pattern[:a]
        t = 0

        for i in range(len(pattern) - 1):
            if self.relMat[pattern[i]][pattern[i+1]] == -1:
                return INT_MAX
            t += self.relMat[pattern[i]][pattern[i+1]]
            if self.nodeType[pattern[i + 1]] == 0:
                minm = max(minm, t)
                t = 0
        if self.relMat[pattern[-1]][pattern[0]] == -1:
            return INT_MAX
        t += self.relMat[pattern[-1]][pattern[0]]
        minm = max(minm, t)
        return minm

    def makeSchedule(self, pattern):
        a = 0
        for i in range(len(pattern)):
            if self.nodeType[pattern[i]] == 0:
                a = i
                break
        pattern = pattern[a:] + pattern[:a]
        k, l = 0, 0
        assignments = dict()
        for i in range(len(pattern)):
            if (i+1 < len(pattern) and self.nodeType[pattern[i + 1]] == 0) or i + 1 == len(pattern):
                l = i
                assignments[pattern[k]] = [pattern[k+1: l+1][0]-self.h]
                k, l = i+1, i+1
        return assignments

    def Schedule(self):
        id = 0
        self.paths = self.HamCycle()
        self.Search()

        for i in range(len(self.paths)):
            self.pattern = self.paths[i]
            span = self.getMakespan(self.pattern)
            if span < self.spanMin:
                self.spanMin = span
                id = i

        if self.spanMin == INT_MAX:
            print("CBS Algorithm Results:")
            print("No feasible solution found")
            return

        temp = self.makeSchedule(self.paths[id])
        print("CBS Algorithm Results:")
        print("Approximate makespan: ", self.spanMin)
        print("Task Allocation Status\n")
        for i in temp:
            print("For robot: "+str(i) + " task assigned is  "+str(temp[i]))

r = CBSRobo(blocked, roboSrc, tasks, nodeData, nodeType, grid_param)
r.distMinAtoB((0,0),(4,6))
r.Schedule()