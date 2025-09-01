import numpy as np
from collections import defaultdict
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

class DStarNode:
    def __init__(self, pos):
        self.pos = pos
        self.g = float('inf')
        self.rhs = float('inf')
        
class DStarRobo:

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
        self.nodes = {}
        self.priority_queue = []

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

    def calculateKey(self, pos, start):
        node = self.getNode(pos)
        return (min(node.g, node.rhs) + self.manhattanDistance(pos, start), 
                min(node.g, node.rhs))

    def getNode(self, pos):
        if pos not in self.nodes:
            self.nodes[pos] = DStarNode(pos)
        return self.nodes[pos]

    def updateVertex(self, pos, start, goal):
        node = self.getNode(pos)
        
        if pos != goal:
            min_rhs = float('inf')
            for neighbor in self.getNeighbors(pos):
                neighbor_node = self.getNode(neighbor)
                min_rhs = min(min_rhs, neighbor_node.g + 1)
            node.rhs = min_rhs
        
        self.removeFromQueue(pos)
        
        if node.g != node.rhs:
            key = self.calculateKey(pos, start)
            heapq.heappush(self.priority_queue, (key, pos))

    def removeFromQueue(self, pos):
        self.priority_queue = [(k, p) for k, p in self.priority_queue if p != pos]
        heapq.heapify(self.priority_queue)

    def computeShortestPath(self, start, goal):
        while (self.priority_queue and 
               (self.priority_queue[0][0] < self.calculateKey(start, start) or
                self.getNode(start).rhs != self.getNode(start).g)):
            
            k_old, u = heapq.heappop(self.priority_queue)
            k_new = self.calculateKey(u, start)
            
            if k_old < k_new:
                heapq.heappush(self.priority_queue, (k_new, u))
            elif self.getNode(u).g > self.getNode(u).rhs:
                self.getNode(u).g = self.getNode(u).rhs
                for neighbor in self.getNeighbors(u):
                    self.updateVertex(neighbor, start, goal)
            else:
                self.getNode(u).g = float('inf')
                self.updateVertex(u, start, goal)
                for neighbor in self.getNeighbors(u):
                    self.updateVertex(neighbor, start, goal)

    def dstarSearch(self, start, goal):
        self.nodes.clear()
        self.priority_queue.clear()
        
        goal_node = self.getNode(goal)
        goal_node.rhs = 0
        
        key = self.calculateKey(goal, start)
        heapq.heappush(self.priority_queue, (key, goal))
        
        self.computeShortestPath(start, goal)
        
        start_node = self.getNode(start)
        if start_node.g == float('inf'):
            return -1
        
        return int(start_node.g)

    def distMinAtoB(self, src, dst):
        self.dist = self.dstarSearch(src, dst)
        
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
        print("D* Algorithm Results:")
        print("Approximate makespan: ", self.spanMin)
        print("Task Allocation Status\n")
        for i in temp:
            print("For robot: "+str(i) + " task assigned is  "+str(temp[i]))

r = DStarRobo(blocked, roboSrc, tasks, nodeData, nodeType, grid_param)
r.distMinAtoB((0,0),(4,6))
r.Schedule()