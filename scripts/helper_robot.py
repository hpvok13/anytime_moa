import heapq
import numpy as np
import math
import copy
import matplotlib.pyplot as plt
from robot_arm import *

INF = 2**32 - 1

class ArmoaNode:
    def __init__(self, state, g, parent, epsilon):
        self.state = state
        self.g = g
        self.f = g + epsilon*state.cell.h
        self.fUnweighted = g + state.cell.h
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
    def __init__(self, pos, h, obstacle):
        self.pos = pos
        self.h = h
        self.obstacle = obstacle

class CellGrid:
    def __init__(self, cellSize, obstacleGrid):
        self.gridHeight = obstacleGrid.shape[0]
        self.gridWidth = obstacleGrid.shape[1]
        assert self.gridWidth % 2 == 0, "obstacleGrid width not even"
        self.height = self.gridHeight*cellSize
        self.width = self.gridWidth*cellSize
        self.minX = -self.width/2
        self.cellSize = cellSize
        self.obstacleGrid = obstacleGrid
        self.grid = np.empty((self.gridHeight, self.gridWidth), dtype=object)
        for i in range(self.gridHeight):
            for j in range(self.gridWidth):
                obstacle = obstacleGrid[i,j] == 1
                x = self.minX + j*cellSize
                y = self.height - (i+1)*cellSize
                self.grid[i, j] = Cell(np.array((x, y)), None, obstacle)
    
    def __getitem__(self, pos, y=None):
        if y == None:
            x = pos[0]
            y = pos[1]
        else:
            x = pos
        i = math.ceil((self.height - y) / self.cellSize)-1
        j = math.floor((x - self.minX) / self.cellSize)
        return self.grid[i, j]
    
    def idxFromPos(self, pos):
        return (math.ceil((self.height - pos[1]) / self.cellSize)-1,  math.floor((pos[0] - self.minX) / self.cellSize))

class State:
    def __init__(self, jointPos, thetas, cell):
        self.jointPos = jointPos
        self.thetas = thetas
        self.cell = cell
        self.gOp = set()
        self.gCl = set()

class StateDict:
    def __init__(self):
        self.dict = dict()

    def __getitem__(self, thetas):
        try:
            return self.dict[tuple(thetas)]
        except KeyError:
            return None
    
    def add(self, state):
        self.dict[tuple(state.thetas)] = state

class RobotProblem:
    def __init__(self, thetas, posGoal, obstacleGrid, robot, cellSize):
        self.gridHeight = obstacleGrid.shape[0]
        self.gridWidth = obstacleGrid.shape[1]
        self.gridSize = cellSize
        self.posGoal = posGoal
        self.obstacleGrid = obstacleGrid
        self.robot = robot
        self.cellSize = cellSize
        self.cellGrid = CellGrid(cellSize, obstacleGrid)
        self.stateDict = StateDict()

        jointPos = robot.fk(thetas)
        cellStart = self.cellGrid[jointPos[self.robot.n,:]]
        self.sStart = State(jointPos, thetas, cellStart)
        self.stateDict.add(self.sStart)
        
        self.distHeuristic = self.calculateDistHeuristic()
        #self.energyHeuristic = self.calculateEnergyHeuristic()

        for i in range(self.gridHeight):
            for j in range(self.gridWidth):
                self.cellGrid.grid[i, j].h = np.array((self.distHeuristic[i, j], 0))

    def getCell(self, pos):
        return self.cellGrid[pos]
    
    def getStartState(self):
        return self.sStart

    def getGoalCell(self):
        return self.getCell(self.posGoal)
    
    def getSuccessors(self, s):
        neighbors = self.robot.getNeighbors(s.thetas, self)
        successors = []
        for thetas, jointPos in neighbors:
            t = self.stateDict[thetas]
            if t == None:
                cell = self.cellGrid[jointPos[self.robot.n,:]]
                if cell.obstacle:
                    continue
                t = State(jointPos, thetas, cell)
                self.stateDict.add(t)
            successors.append(t)
        return successors

    def getNeighborsPos(self, pos):
        neighbors = []
        for i in range(pos[0] - 1, pos[0] + 2):
            for j in range(pos[1] - 1, pos[1] + 2):
                if ((i, j) == pos
                    or i < 0 or i >= self.gridHeight
                    or j < 0 or j >= self.gridWidth
                    or self.obstacleGrid[i, j] == 1):
                    continue
                else:
                    neighbors.append((i, j))
        return neighbors

    def calculateSafetyHeuristic(self):
        grid = np.ones((self.gridHeight, self.gridWidth), dtype=float)*INF

        for i in range(self.gridHeight):
            for j in range(self.gridWidth):
                open = []
                open.append(np.array((i,j)))
                while len(open) > 0:
                    curNode = open.pop(0)
                    curPos = curNode.pos
                    neighborsPos = self.getNeighborsPos(curPos)
                    for neighborPos in neighborsPos:
                        neighbor = grid[neighborPos]
                        if neighbor.visited:
                            continue

                        if abs(neighborPos[0] - curPos[0]) + abs(neighborPos[1] - curPos[1]) > 1:
                            newDist = curNode.dist + self.cellSize*math.sqrt(2)
                        else:
                            newDist = curNode.dist + self.cellSize

                        if newDist < neighbor.dist:
                            neighbor.dist = newDist
                            open.push(neighbor)
                    curNode.visited = True
        
        distGrid = np.zeros((self.gridHeight, self.gridWidth))
        for i in range(self.gridHeight):
            for j in range(self.gridWidth):
                distGrid[i, j] = grid[i, j].dist

        return distGrid
    
    def calculateDistHeuristic(self):
        grid = np.empty((self.gridHeight, self.gridWidth), dtype=object)
        for i in range(self.gridHeight):
            for j in range(self.gridWidth):
                if self.obstacleGrid[i, j] == 1:
                    grid[i, j] = DijkstraNode((i, j), INF, True)
                else:
                    grid[i, j] = DijkstraNode((i, j), INF, False)
        
        curNode = grid[self.cellGrid.idxFromPos(self.posGoal)]
        curNode.dist = 0
        open = PriorityQueue()
        open.push(curNode)
        while not open.empty():
            curNode = open.pop()
            curPos = curNode.pos
            neighborsPos = self.getNeighborsPos(curPos)
            for neighborPos in neighborsPos:
                neighbor = grid[neighborPos]
                if neighbor.visited:
                    continue

                if abs(neighborPos[0] - curPos[0]) + abs(neighborPos[1] - curPos[1]) > 1:
                    newDist = curNode.dist + self.cellSize*math.sqrt(2)
                else:
                    newDist = curNode.dist + self.cellSize

                if newDist < neighbor.dist:
                    neighbor.dist = newDist
                    open.push(neighbor)
            curNode.visited = True
        
        distGrid = np.zeros((self.gridHeight, self.gridWidth))
        for i in range(self.gridHeight):
            for j in range(self.gridWidth):
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
        x.f = x.g + epsilon*x.state.cell.h
    heapq.heapify(openList)

def updateFOpenIncon(openList, epsilon):
    for x in openList:
        x.f = x.g + epsilon*x.state.cell.h
        x.state.gOp.add(x)
    heapq.heapify(openList)

def publishSolutions(sols, problem, doPrint):
    paths = []
    print(str(len(sols)) + " Solutions")
    for sol in sols:
        nTmp = sol
        path = []
        while nTmp != None:
            path.append(nTmp.state.jointPos)
            nTmp = nTmp.parent
        path.reverse()
        if doPrint:
            problem.robot.visualizeTrajectory(path, problem)
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