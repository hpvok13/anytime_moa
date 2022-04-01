import heapq
import numpy as np
import math
import matplotlib.pyplot as plt
from robot_arm import *

INF = 2**32 - 1
# MAX_TURN_TICKS = 4

class ArmoaNode:
    def __init__(self, state, g, parent, epsilon):
        self.state = state
        self.g = g
        self.f = g + epsilon*state.h
        self.fUnweighted = g + state.h
        self.parent = parent

    def __lt__(self, other):
        return lexiLT(self.f, other.f)

    def dominates(self, other):
        return dominates(self.f, other.f)

class NamoaNode:
    def __init__(self, state, g, parent):
        self.state = state
        self.g = g
        self.f = g + state.h
        self.parent = parent

    def __lt__(self, other):
        return lexiLT(self.f, other.f)

    def dominates(self, other):
        return dominates(self.f, other.f)

class Obstacle:
    def __init__(self, center, radius):
        self.center = np.array(center)
        self.radius = radius

    def pointIn(self, point):
        return np.linalg.norm(np.array(point) - self.center) < self.radius

# class ObstacleGrid:
#     def __init__(self, height, width, statesPerMeter, obstacles):
#         self.height = height
#         self.width = width
#         self.statesPerMeter = statesPerMeter
#         self.cellSize = 1/statesPerMeter
#         gridHeight = math.ceil(height*statesPerMeter)
#         gridWidth = math.ceil(width*statesPerMeter)
#         self.grid = np.zeros((gridHeight, gridWidth))
#         for i in range(gridHeight):
#             for j in range(gridWidth):
#                 for obstacle in obstacles:
#                     x = i*self.cellSize
#                     y = j*self.cellSize
#                     pos = np.array((x, y))
#                     if obstacle.pointIn(pos):
#                         self.grid[i, j] = 1
#                         break
    
#     def __getitem__(self, pos, y=None):
#         if y == None:
#             x = pos[0]
#             y = pos[1]
#         else:
#             x = pos
#         i = math.floor(x*self.statesPerMeter)
#         j = math.floor(y*self.statesPerMeter)
#         return self.grid[i, j]
    
#     def idxFromPos(self, pose):
#         return (math.floor(pose[0]*self.statesPerMeter), 
#                 math.floor(pose[1]*self.statesPerMeter)) 

#     def getNeighborsPos(self, pos):
#         neighbors = []
#         for i in range(pos[0] - 1, pos[0] + 2):
#             for j in range(pos[1] - 1, pos[1] + 2):
#                 if ((i, j) == pos
#                     or i < 0 or i >= self.grid.shape[0]
#                     or j < 0 or j >= self.grid.shape[1]
#                     or self.grid[i, j] == 1):
#                     continue
#                 else:
#                     neighbors.append((i, j))
#         assert len(neighbors) != 0, 'No neighbors in obstacle grid'
#         return neighbors

class State:
    def __init__(self, idx, pose, h):
        self.idx = idx
        self.pose = pose
        self.gOp = set()
        self.gCl = set()
        self.h = h

class StateGrid:
    def __init__(self, height, width, numThetas, statesPerMeter, obstacles, maxTurnTicks):
        self.height = height
        self.width = width
        self.numThetas = numThetas
        self.statesPerMeter = statesPerMeter
        self.cellSize = 1/statesPerMeter
        self.maxTurnTicks = maxTurnTicks
        gridHeight = math.ceil(height*statesPerMeter)
        gridWidth = math.ceil(width*statesPerMeter)
        self.obstacles = obstacles
        self.grid = np.empty((gridHeight, gridWidth, numThetas), dtype=object)
        self.shape = self.grid.shape
        for i in range(gridHeight):
            for j in range(gridWidth):
                for k in range(numThetas):
                    x = i*self.cellSize
                    y = j*self.cellSize
                    theta = -math.pi + k*2*math.pi/numThetas
                    pose = np.array((x, y, theta))
                    self.grid[i, j, k] = State((i, j, k), pose, None)
    
    def __getitem__(self, pose, y=None, theta=None):
        if y == None:
            x = pose[0]
            y = pose[1]
            theta = pose[2]
        else:
            x = pose
        i = math.floor(x*self.statesPerMeter)
        j = math.floor(y*self.statesPerMeter)
        k = math.floor((theta + math.pi)*self.numThetas/(2*math.pi))
        return self.grid[i, j, k]
    
    def idxFromPose(self, pose):
        return (math.floor(pose[0]*self.statesPerMeter), 
                math.floor(pose[1]*self.statesPerMeter), 
                math.floor((pose[2] + math.pi)*self.numThetas/(2*math.pi)))

    def poseFromIdx(self, idx):
        x = idx[0]*self.cellSize
        y = idx[1]*self.cellSize
        theta = -math.pi + idx[2]*2*math.pi/self.numThetas
        return np.array((x, y, theta))

    def idxFromPos(self, pose):
        return (math.floor(pose[0]*self.statesPerMeter), 
                math.floor(pose[1]*self.statesPerMeter))

    def posFromIdx(self, idx):
        x = idx[0]*self.cellSize
        y = idx[1]*self.cellSize
        return np.array((x, y))

    def getSuccessors(self, s):
        neighbors = []
        idx = s.idx
        for i in range(idx[0] - 1, idx[0] + 2):
            for j in range(idx[1] - 1, idx[1] + 2):
                if (i < 0 or i >= self.shape[0]
                    or j < 0 or j >= self.shape[1]):
                    continue
                neighborPos = self.posFromIdx((i, j))
                for obstacle in self.obstacles:
                    if obstacle.pointIn(neighborPos):
                        break
                else:
                    for k in range(idx[2] - 1, idx[2] + 2):
                        kMod = k % self.numThetas
                        successorIdx = (i, j, kMod)
                        if successorIdx == idx:
                            continue
                        else:
                            neighbors.append(self.grid[successorIdx])
        assert len(neighbors) != 0, 'No neighbors state grid'
        for neighbor in neighbors:
            assert not obstacle.pointIn(neighbor.pose[:2])
        return neighbors

    def getNeighborsIdx(self, idx):
        neighbors = []
        for i in range(idx[0] - 1, idx[0] + 2):
            for j in range(idx[1] - 1, idx[1] + 2):
                if (i < 0 or i >= self.shape[0]
                    or j < 0 or j >= self.shape[1]):
                    continue
                neighborPos = self.posFromIdx((i, j))
                for obstacle in self.obstacles:
                    if obstacle.pointIn(neighborPos):
                        break
                else:
                    for k in range(idx[2] - 1, idx[2] + 2):
                        kMod = k % self.numThetas
                        neighborIdx = (i, j, kMod)
                        if neighborIdx == idx:
                            continue
                        else:
                            neighbors.append(neighborIdx)
        assert len(neighbors) != 0, 'No neighbors state grid'
        return neighbors

def subtractAngles(theta1, theta2):
    assert -math.pi <= theta1 and theta1 < math.pi
    assert -math.pi <= theta2 and theta2 < math.pi
    diff = theta1 - theta2
    if diff > math.pi:
        diff = -2*math.pi + diff
    elif diff < -math.pi:
        diff = 2*math.pi + diff
    assert -math.pi <= diff and diff < math.pi
    return diff

def addAngles(theta1, theta2):
    assert -math.pi <= theta1 and theta1 < math.pi
    assert -math.pi <= theta2 and theta2 < math.pi
    sum = theta1 + theta2
    if sum > math.pi:
        sum = -2*math.pi + sum
    elif sum < -math.pi:
        sum = 2*math.pi + sum
    assert -math.pi <= sum and sum < math.pi
    return sum

def getNeighborsIdx2D(idx, shape):
    neighbors = []
    for i in range(idx[0] - 1, idx[0] + 2):
        for j in range(idx[1] - 1, idx[1] + 2):
            neighborIdx = (i, j)
            if (i < 0 or i >= shape[0]
                or j < 0 or j >= shape[1]
                or neighborIdx == idx):
                continue
            neighbors.append(neighborIdx)
    return neighbors

class ShipBotProblem:
    def __init__(self, poseStart, poseGoal, height, width, numThetas, obstacles, robot, statesPerMeter, maxTurnTicks, calculateHeuristic):
        self.height = height
        self.width = width
        self.poseStart = poseStart
        self.poseGoal = poseGoal
        self.robot = robot
        self.statesPerMeter = statesPerMeter
        self.obstacles = obstacles
        self.stateGrid = StateGrid(height, width, numThetas, statesPerMeter, obstacles, maxTurnTicks)
        #self.obstacleGrid = ObstacleGrid(height, width, statesPerMeter, obstacles)

        self.sStart = self.stateGrid[poseStart]
        
        if calculateHeuristic:
            self.distHeuristic = self.calculate3DDistHeuristic()
            self.safeHeuristic, self.safeCost = self.calculateSafeHeuristic()
            np.save('distHeuristic.npy', self.distHeuristic)
            np.save('safeHeuristic.npy', self.safeHeuristic)
            np.save('safeCost.npy', self.safeCost)
        else:
            try:
                self.distHeuristic = np.load('distHeuristic.npy')
            except OSError:
                self.distHeuristic = self.calculate3DDistHeuristic()
                np.save('distHeuristic.npy', self.distHeuristic)
            try:
                self.safeHeuristic = np.load('safeHeuristic.npy')
                self.safeCost = np.load('safeCost.npy')
            except OSError:
                self.safeHeuristic, self.safeCost = self.calculateSafeHeuristic()
                np.save('safeHeuristic.npy', self.safeHeuristic)
                np.save('safeCost.npy', self.safeCost)

        plt.imshow(self.safeCost)
        plt.show()

        for i in range(self.stateGrid.shape[0]):
            for j in range(self.stateGrid.shape[1]):
                for k in range(self.stateGrid.shape[2]):
                    self.stateGrid.grid[i, j, k].h = np.array((self.distHeuristic[i, j, k], self.safeHeuristic[i, j, k]))

    def getState(self, pose):
        return self.stateGrid[pose]
    
    def getStartState(self):
        return self.sStart

    def getGoalState(self):
        return self.getState(self.poseGoal)
    
    def getSuccessors(self, s):
        return self.stateGrid.getSuccessors(s)

    def calculateSafeHeuristic(self):
        grid = np.empty(self.stateGrid.shape[:2], dtype=object)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                idx = (i, j)
                pos = self.stateGrid.posFromIdx(idx)
                grid[idx] = DijkstraNode(idx, pos, INF, False)
        
        for first, obstacle in enumerate(self.obstacles):
            if first != 0:
                for i in range(grid.shape[0]):
                    for j in range(grid.shape[1]):
                        grid[i, j].visited = False

            curNode = grid[self.stateGrid.idxFromPos(obstacle.center)]
            curNode.dist = 0
            open = PriorityQueue()
            open.push(curNode)
            while not open.empty():
                curNode = open.pop()
                curIdx = curNode.idx
                curPos = curNode.pose
                neighborsIdx = getNeighborsIdx2D(curIdx, grid.shape)
                for neighborIdx in neighborsIdx:
                    neighbor = grid[neighborIdx]
                    neighborPos = neighbor.pose
                    if neighbor.visited:
                        continue

                    ## Subtraction of angles requires checks
                    newDist = curNode.dist + np.linalg.norm(neighborPos - curPos)

                    if newDist < neighbor.dist:
                        neighbor.dist = newDist
                        open.push(neighbor)
                curNode.visited = True
        
        costGrid = np.zeros(grid.shape)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                costGrid[i, j] = grid[i, j].dist

                
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                grid[i, j].dist= INF
                for obstacle in self.obstacles:
                    if obstacle.pointIn(grid[i, j].pose):
                        grid[i, j].visited = True
                        break
                else:
                    grid[i, j].visited = False
        
        curNode = grid[self.stateGrid.idxFromPos(self.poseGoal[:2])]
        curNode.dist = 0
        open = PriorityQueue()
        open.push(curNode)
        while not open.empty():
            curNode = open.pop()
            curIdx = curNode.idx
            curPos = curNode.pose
            neighborsIdx = getNeighborsIdx2D(curIdx, grid.shape)
            for neighborIdx in neighborsIdx:
                neighbor = grid[neighborIdx]
                neighborPos = neighbor.pose
                if neighbor.visited:
                    continue

                ## Subtraction of angles requires checks
                newDist = curNode.dist + costGrid[neighborIdx]

                if newDist < neighbor.dist:
                    neighbor.dist = newDist
                    open.push(neighbor)
            curNode.visited = True
        
        distGrid = np.zeros(self.stateGrid.shape)
        for i in range(distGrid.shape[0]):
            for j in range(distGrid.shape[1]):
                for k in range(distGrid.shape[2]):
                    distGrid[i, j, k] = grid[i, j].dist

        return distGrid, costGrid
    
    def calculate3DDistHeuristic(self):
        grid = np.empty(self.stateGrid.shape, dtype=object)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                isObstacle = False
                neighborPos = self.stateGrid.posFromIdx((i, j))
                for obstacle in self.obstacles:
                    if obstacle.pointIn(neighborPos):
                        isObstacle = True
                        break
                for k in range(grid.shape[2]):
                    idx = (i, j, k)
                    pose = self.stateGrid.poseFromIdx(idx)
                    if isObstacle:
                        grid[i, j, k] = DijkstraNode(idx, pose, INF, True)
                    else:
                        grid[i, j, k] = DijkstraNode(idx, pose, INF, False)
        
        curNode = grid[self.stateGrid.idxFromPose(self.poseGoal)]
        curNode.dist = 0
        open = PriorityQueue()
        open.push(curNode)
        while not open.empty():
            curNode = open.pop()
            curIdx = curNode.idx
            curPose = curNode.pose
            neighborsIdx = self.stateGrid.getNeighborsIdx(curIdx)
            for neighborIdx in neighborsIdx:
                neighbor = grid[neighborIdx]
                neighborPose = neighbor.pose
                if neighbor.visited:
                    continue

                ## Subtraction of angles requires checks
                newDist = curNode.dist + np.linalg.norm(np.array([neighborPose[0] - curPose[0], 
                                                                  neighborPose[1] - curPose[1], 
                                                                  subtractAngles(neighborPose[2], curPose[2])]))

                if newDist < neighbor.dist:
                    neighbor.dist = newDist
                    open.push(neighbor)
            curNode.visited = True
        
        distGrid = np.zeros(grid.shape)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                for k in range(grid.shape[2]):
                    distGrid[i, j, k] = grid[i, j, k].dist

        return distGrid

class DijkstraNode:
    def __init__(self, idx, pose, dist, visited):
        self.idx = idx
        self.pose = pose
        self.dist = dist
        self.visited = visited
    
    def __lt__(self, other):
        return self.dist < other.dist

class PriorityQueue:
    def __init__(self, list=None):
        if list == None:
            self.heap = []
        else:
            self.heap = list
            heapq.heapify(self.heap)

    def empty(self):
        return len(self.heap) == 0

    def push(self, x):
        heapq.heappush(self.heap, x)

    def pop(self):
        return heapq.heappop(self.heap)
                    
    def peek(self):
        return self.heap[0]

    def updateOrder(self):
        heapq.heapify(self.heap)

    def mergeUpdate(self, list):
        self.heap = self.heap + list
        self.updateOrder()

def lexiLT(a, b):
    return (a[0] < b[0]
            or (a[0] == b[0]
            and a[1] < b[1]))

def dominates(a, b):
    return ((a[0] < b[0] and a[1] <= b[1])
            or (a[0] == b[0] and a[1] < b[1]))

def setDominates(set, vector):
    for x in set:
        if dominates(x.g, vector):
            return True
    return False

def setDominated(set, vector):
    for x in set:
        if dominates(vector, x.g):
            return True
    return False

def shouldIterate(sols, set):
    for x in set:
        if not setDominates(sols, x.fUnweighted):
            return True
    return False

def filterOpenArmoa(vector, openList):
    removeList = []
    for x in openList:
        if dominates(vector, x.fUnweighted):
            removeList.append(x)
    for x in removeList:
        openList.remove(x)
        x.state.gOp.remove(x)
    heapq.heapify(openList)

def filterOpenNamoa(vector, openList):
    removeList = []
    for x in openList:
        if dominates(vector, x.f):
            removeList.append(x)
    for x in removeList:
        openList.remove(x)
        x.state.gOp.remove(x)
    heapq.heapify(openList)

def filterGOp(vector, gOp, openList):
    removeList = []
    for x in gOp:
        if dominates(vector, x.g):
            removeList.append(x)
    for x in removeList:
        gOp.remove(x)
        openList.remove(x)
    heapq.heapify(openList)

def filterSet(vector, set):
    removeList = []
    for x in set:
        if dominates(vector, x.g):
            removeList.append(x)
    for x in removeList:        
        set.remove(x)

def shouldTerminate(sols, openList):
    for x in openList:
        if not setDominates(sols, x.f):
            return False
    return True

def updateFOpen(openList, epsilon):
    for x in openList:
        x.f = x.g + epsilon*x.state .h
    heapq.heapify(openList)

def updateFOpenIncon(openList, epsilon):
    for x in openList:
        x.f = x.g + epsilon*x.state.h
        x.state.gOp.add(x)
    heapq.heapify(openList)

# def publishSolutions(sols, problem):
#     paths = []
#     print(str(len(sols)) + " Solutions")
#     for sol in sols:
#         im = np.zeros(problem.stateGrid.shape[:2])
#         nTmp = sol
#         path = []
#         while nTmp != None:
#             path.append(nTmp.state.pose)
#             im[tuple(nTmp.state.idx[:2])] = 1
#             nTmp = nTmp.parent
#         path.reverse()
#         paths.append(path)
#         plt.imshow(im)
#         plt.show()
#     return paths

def publishSolutions(sols, problem):
    paths = []
    print(str(len(sols)) + " Solutions")
    for sol in sols:
        _, ax = plt.subplots()
        nTmp = sol
        path = []
        while nTmp != None:
            pose = nTmp.state.pose
            x = pose[0]
            y = pose[1]
            dx = math.cos(pose[2])*0.1
            dy = math.sin(pose[2])*0.1
            path.append(nTmp.state.pose)
            ax.arrow(x, y, dx, dy)
            nTmp = nTmp.parent
        path.reverse()
        paths.append(path)
        plt.show()
    return paths

def verifyPaths(sols1, sols2):
    if len(sols1) != len(sols2):
        print("Pareto optimal solution sets have different length")
        return False

    for sol2 in sols2:
        for sol1 in sols1:
            if np.array_equal(sol1.g, sol2.g):
                break
        else:
            break
    else:
        return True
    print("Solutions don't match")
    return False

def publishPathCosts(sols):
    x = []
    y = []
    for sol in sols:
        x.append(sol.g[0])
        y.append(sol.g[1])
    plt.scatter(x,y)