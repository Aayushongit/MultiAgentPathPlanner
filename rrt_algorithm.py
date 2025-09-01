import numpy as np
from collections import defaultdict
import random
import math
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

class RRTNode:
    def __init__(self, pos):
        self.pos = pos
        self.parent = None

class RRTRobo:

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
        self.step_size = 1
        self.max_iterations = 1000

    def getPos(self):
        return (self.x, self.y)

    def euclideanDistance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def isValidPoint(self, pos):
        x, y = pos
        return (0 <= x < self.x and 0 <= y < self.y and pos not in self.blocked)

    def getRandomPoint(self):
        return (random.randint(0, self.x-1), random.randint(0, self.y-1))

    def findNearestNode(self, nodes, target_pos):
        min_dist = float('inf')
        nearest_node = None
        
        for node in nodes:
            dist = self.euclideanDistance(node.pos, target_pos)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                
        return nearest_node

    def steer(self, from_pos, to_pos):
        dist = self.euclideanDistance(from_pos, to_pos)
        if dist <= self.step_size:
            return to_pos
        
        angle = math.atan2(to_pos[1] - from_pos[1], to_pos[0] - from_pos[0])
        new_x = from_pos[0] + self.step_size * math.cos(angle)
        new_y = from_pos[1] + self.step_size * math.sin(angle)
        
        return (int(round(new_x)), int(round(new_y)))

    def isObstacleFree(self, from_pos, to_pos):
        x0, y0 = from_pos
        x1, y1 = to_pos
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            if (x0, y0) in self.blocked:
                return False
                
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return True

    def rrtSearch(self, start, goal):
        start_node = RRTNode(start)
        nodes = [start_node]
        
        for _ in range(self.max_iterations):
            if random.random() < 0.1:
                random_pos = goal
            else:
                random_pos = self.getRandomPoint()
                
            if not self.isValidPoint(random_pos):
                continue
                
            nearest_node = self.findNearestNode(nodes, random_pos)
            new_pos = self.steer(nearest_node.pos, random_pos)
            
            if not self.isValidPoint(new_pos):
                continue
                
            if not self.isObstacleFree(nearest_node.pos, new_pos):
                continue
                
            new_node = RRTNode(new_pos)
            new_node.parent = nearest_node
            nodes.append(new_node)
            
            if new_pos == goal:
                path = []
                current = new_node
                while current is not None:
                    path.append(current.pos)
                    current = current.parent
                return len(path) - 1
                
        return -1

    def distMinAtoB(self, src, dst):
        self.dist = self.rrtSearch(src, dst)
        
        if self.dist == -1:
            print('no possible path from '+str(src)+" to "+str(dst))
        
        return self.dist

    def Search(self):
        for i in range(self.w+self.h):
            for j in range(self.w+self.h):
                robot, task = nodeType[i], nodeType[j]
                if robot == 0 and task == 1:
                    self.relMat[i][j] = self.distMinAtoB(self.nodeData[i][0], self.nodeData[j][0])
                elif robot == 1 and task == 1:
                    self.relMat[i][j] = self.distMinAtoB(self.nodeData[i][0], self.nodeData[i][1]) + self.distMinAtoB(self.nodeData[i][1], self.nodeData[j][0])
                elif robot == 1 and task == 0:
                    self.relMat[i][j] = self.distMinAtoB(self.nodeData[i][0], self.nodeData[i][1]) + self.distMinAtoB(self.nodeData[i][1], self.nodeData[j][1])
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
            t += self.relMat[pattern[i]][pattern[i+1]]
            if self.nodeType[pattern[i + 1]] == 0:
                minm = max(minm, t)
                t = 0
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

        temp = self.makeSchedule(self.paths[id])
        print("RRT Algorithm Results:")
        print("Approximate makespan: ", self.spanMin)
        print("Task Allocation Status\n")
        for i in temp:
            print("For robot: "+str(i) + " task assigned is  "+str(temp[i]))

r = RRTRobo(blocked, roboSrc, tasks, nodeData, nodeType, grid_param)
r.distMinAtoB((0,0),(4,6))
r.Schedule()