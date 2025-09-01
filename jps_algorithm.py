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

class JPSRobo:

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

    def isValidPoint(self, pos):
        x, y = pos
        return (0 <= x < self.x and 0 <= y < self.y and pos not in self.blocked)

    def getDirections(self):
        return [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def jump(self, current, direction, goal):
        x, y = current
        dx, dy = direction
        
        if not self.isValidPoint(current):
            return None
            
        if current == goal:
            return current
            
        next_pos = (x + dx, y + dy)
        
        if not self.isValidPoint(next_pos):
            return None
            
        if dx != 0 and dy != 0:
            if (self.jump(next_pos, (dx, 0), goal) is not None or 
                self.jump(next_pos, (0, dy), goal) is not None):
                return next_pos
        else:
            if dx != 0:
                if ((self.isValidPoint((x, y + 1)) and not self.isValidPoint((x - dx, y + 1))) or
                    (self.isValidPoint((x, y - 1)) and not self.isValidPoint((x - dx, y - 1)))):
                    return current
            else:
                if ((self.isValidPoint((x + 1, y)) and not self.isValidPoint((x + 1, y - dy))) or
                    (self.isValidPoint((x - 1, y)) and not self.isValidPoint((x - 1, y - dy)))):
                    return current
        
        return self.jump(next_pos, direction, goal)

    def getSuccessors(self, current, goal):
        successors = []
        
        for direction in self.getDirections():
            jump_point = self.jump(current, direction, goal)
            if jump_point is not None:
                successors.append(jump_point)
                
        return successors

    def jpsSearch(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattanDistance(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                path_length = 0
                while current in came_from:
                    path_length += 1
                    current = came_from[current]
                return path_length
            
            for successor in self.getSuccessors(current, goal):
                tentative_g_score = g_score[current] + self.manhattanDistance(current, successor)
                
                if successor not in g_score or tentative_g_score < g_score[successor]:
                    came_from[successor] = current
                    g_score[successor] = tentative_g_score
                    f_score[successor] = g_score[successor] + self.manhattanDistance(successor, goal)
                    
                    if (f_score[successor], successor) not in open_set:
                        heapq.heappush(open_set, (f_score[successor], successor))
        
        return -1

    def distMinAtoB(self, src, dst):
        self.dist = self.jpsSearch(src, dst)
        
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
        print("JPS Algorithm Results:")
        print("Approximate makespan: ", self.spanMin)
        print("Task Allocation Status\n")
        for i in temp:
            print("For robot: "+str(i) + " task assigned is  "+str(temp[i]))

r = JPSRobo(blocked, roboSrc, tasks, nodeData, nodeType, grid_param)
r.distMinAtoB((0,0),(4,6))
r.Schedule()