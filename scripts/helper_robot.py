import heapq
import numpy as np
from PIL import Image as im
import copy
import matplotlib.pyplot as plt

INF = 2**32 - 1

class CellGrid:
    def __init__(self, gridSize, obstacleGrid):
        self.gridHeight = obstacleGrid.shape[0]
        self.gridWidth = obstacleGrid.shape[1]
        assert self.gridWidth % 2 == 0, "obstacleGrid width not even"
        self.height = self.gridHeight*gridSize
        self.width = self.gridWidth*gridSize
        self.minX = -self.width/2
        self.gridSize = gridSize
        self.obstacleGrid = obstacleGrid
        self.grid = np.empty((self.gridHeight, self.gridWidth), dtype=object)
        for i in range(self.height):
            for j in range(self.width):

                self.grid[i, j] = Cell((x, y), None, None)
    
    def __getitem__(self, x, y):
        i = (y-/self.gridSize 
        return self.grid[]


class ArmoaNode:
    def __init__(self, cell, thetas, g, parent, epsilon):
        self.state = thetas
        self.cell = cell
        self.g = g
        self.f = g + epsilon*state.h
        self.fUnweighted = g + state.h
        self.parent = parent

    def __lt__(self, other):
        return lexiLT(self.f, other.f)

    def dominates(self, other):
        return dominates(self.f, other.f)

class NamoaNode:
    def __init__(self, state, thetas, g, parent):
        self.state = state
        self.thetas = thetas
        self.g = g
        self.f = g + state.h
        self.parent = parent

    def __lt__(self, other):
        return lexiLT(self.f, other.f)

    def dominates(self, other):
        return dominates(self.f, other.f)

class Cell:
    def __init__(self, pos, h):
        self.pos = pos
        self.h = h

class State:
    def __init__(self, thetas, h):
        self.thetas = thetas
        self.gOp = set()
        self.gCl = set()
        self.h = h

class RobotProblem:
    def __init__(self, cellStart, cellGoal, obstacleGrid, robot, gridSize):
        self.height = obstacleGrid.shape[0]
        self.width = obstacleGrid.shape[1]
        self.gridSize = gridSize
        self.cellGrid = np.empty((self.height, self.width), dtype=object)
        self.posStart = cellStart
        self.posGoal = cellGoal
        self.obstacleGrid = obstacleGrid
        self.robot = robot
        for i in range(self.height):
            for j in range(self.width):
                self.grid[i, j] = Cell((i, j), None)
        
        self.distHeuristic = self.calculateDistHeuristic()
        self.energyHeuristic = self.heuristicGrid(self.costGrid2)

        for i in range(self.height):
            for j in range(self.width):
                if obstacleGrid[i, j] == 1:
                    continue
                else:
                    self.grid[i, j].h = np.array((self.hGrid1[i, j], self.hGrid2[i, j]))
    
    def getState(self, pos):
        return self.grid[pos]
    
    def getStart(self):
        return self.getState(self.posStart)

    def getGoal(self):
        return self.getState(self.posGoal)
    
    def getSuccessors(self, s):
        self.robot

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
    
    def calculateDistHeuristic(self):
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
        while not open.empty():
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
        x.f = x.g + epsilon*x.state.h
    heapq.heapify(openList)

def updateFOpenIncon(openList, epsilon):
    for x in openList:
        x.f = x.g + epsilon*x.state.h
        x.state.gOp.add(x)
    heapq.heapify(openList)

def createCostGrids(height, width, fname1, fname2):
    costGrid1 = np.random.randint(1, 10, (height, width))
    costGrid2 = np.random.randint(1, 10, (height, width))
    np.savetxt(fname1, costGrid1, delimiter=',')
    np.savetxt(fname2, costGrid2, delimiter=',')

def publishPath(path, obstacleGrid):
    grid = copy.deepcopy(obstacleGrid)
    grid = -(grid - 1) * 127

    for pos in path:
        grid[pos] = 255
    
    # image = im.fromarray(grid)
    # image.show()
    plt.imshow(grid)
    plt.show()

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