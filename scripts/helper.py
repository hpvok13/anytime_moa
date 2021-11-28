import heapq
import numpy as np
from PIL import Image as im
import copy
import matplotlib.pyplot as plt

INF = 2**32 - 1

class MoaraNode:
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

class State:
    def __init__(self, pos, h, cost):
        self.pos = pos
        self.gOp = set()
        self.gCl = set()
        self.h = h
        self.cost = cost

class GridProblem:
    def __init__(self, posStart, posGoal, obstacleGrid):
        self.height = obstacleGrid.shape[0]
        self.width = obstacleGrid.shape[1]
        self.grid = np.empty((self.height, self.width), dtype=object)
        self.posStart = posStart
        self.posGoal = posGoal
        self.obstacleGrid = obstacleGrid
        for i in range(self.height):
            for j in range(self.width):
                self.grid[i, j] = State((i, j), None, None)
        self.costGrid1 = np.random.randint(1, 10, (self.height, self.width))
        self.costGrid2 = np.random.randint(1, 10, (self.height, self.width))
        self.hGrid1 = self.heuristicGrid(self.costGrid1)
        self.hGrid2 = self.heuristicGrid(self.costGrid2)
        for i in range(self.height):
            for j in range(self.width):
                if obstacleGrid[i, j] == 1:
                    continue
                else:
                    self.grid[i, j].h = np.array((self.hGrid1[i, j], self.hGrid2[i, j]))
                    self.grid[i, j].cost = np.array((self.costGrid1[i, j], self.costGrid2[i, j]))
    
    def getState(self, pos):
        return self.grid[pos]
    
    def getStart(self):
        return self.getState(self.posStart)

    def getGoal(self):
        return self.getState(self.posGoal)
    
    def getSuccessors(self, s):
        successors = []
        for i in range(s.pos[0] - 1, s.pos[0] + 2):
            for j in range(s.pos[1] - 1, s.pos[1] + 2):
                if ((i, j) == s.pos
                    or i < 0 or i >= self.height
                    or j < 0 or j >= self.width
                    or self.obstacleGrid[i, j] == 1):
                    continue
                else:
                    successors.append(self.grid[i, j])
        return successors

    def getNeighborsPos(self, pos):
        neighbors = []
        for i in range(pos[0] - 1, pos[0] + 2):
            for j in range(pos[1] - 1, pos[1] + 2):
                if ((i, j) == pos
                    or i < 0 or i >= self.height
                    or j < 0 or j >= self.width
                    or self.obstacleGrid[i, j] == 1):
                    continue
                else:
                    neighbors.append((i, j))
        return neighbors
    
    def heuristicGrid(self, costGrid):
        grid = np.empty((self.height, self.width), dtype=object)
        for i in range(self.height):
            for j in range(self.width):
                if self.obstacleGrid[i, j] == 1:
                    grid[i, j] = DijkstraNode((i, j), INF, True)
                else:
                    grid[i, j] = DijkstraNode((i, j), INF, False)
        
        curNode = grid[self.posGoal]
        curNode.dist = 0
        open = PriorityQueue()
        open.push(curNode)
        while not open.empty():############
            curNode = open.pop()
            curPos = curNode.pos
            newDist = curNode.dist + costGrid[curPos]
            neighborsPos = self.getNeighborsPos(curPos)
            for neighborPos in neighborsPos:
                neighbor = grid[neighborPos]
                if neighbor.visited:
                    continue
                if newDist < neighbor.dist:
                    neighbor.dist = newDist
                    open.push(neighbor)
            curNode.visited = True
        
        distGrid = np.zeros((self.height, self.width))
        for i in range(self.height):
            for j in range(self.width):
                distGrid[i, j] = grid[i, j].dist

        return distGrid

class DijkstraNode:
    def __init__(self, pos, dist, visited):
        self.pos = pos
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

def shouldIterate(sols, set):
    for x in set:
        if not setDominates(sols, x.fUnweighted):
            return True
    return False

def filterOpenMoara(vector, openList):
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
        x.f = x.g + epsilon*x.state.h
    heapq.heapify(openList)

def publishPath(path, obstacleGrid):
    grid = copy.deepcopy(obstacleGrid)
    grid = -(grid - 1) * 127

    for pos in path:
        grid[pos] = 255
    
    image = im.fromarray(grid)

    image.show()
    # plt.imshow(grid)
    # plt.show()

def publishSolutions(sols, obstacleGrid, doPrint):
    paths = []
    print(str(len(sols)) + " Solutions")
    for sol in sols:
        nTmp = sol
        path = []
        while nTmp != None:
            path.append(nTmp.state.pos)
            nTmp = nTmp.parent
        path.reverse()
        if doPrint:
            publishPath(path, obstacleGrid)
        paths.append(path)
    return paths