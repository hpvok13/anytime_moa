import heapq
import numpy as np
import copy

INF = 2**32 - 1

class Node:
    def __init__(self, state, g, parent):
        self.state = state
        self.g = g
        self.f = g + state.h
        self.parent = parent

    def __lt__(self, other):
        return lexiLT(self.f, other.f)

    def dominates(self, other):
        return dominates(self.f, other.f)

class DijkstraNode:
    def __init__(self, pos, dist, visited):
        self.pos = pos
        self.dist = dist
        self.visited = visited
    
    def __lt__(self, other):
        return self.dist < other.dist

class State:
    def __init__(self, pos, isObs, h, cost):
        self.pos = pos
        self.isObs = isObs
        self.gOp = set()
        self.gCl = set()
        self.h = h
        self.cost = cost

class GridProblem:
    def __init__(self, height, width, posStart, posGoal, obstacles, obstacleGrid):
        self.height = height
        self.width = width
        self.grid = np.empty((height, width), dtype=object)
        self.posStart = posStart
        self.posGoal = posGoal
        self.obstacles = obstacles
        self.obstacleGrid = obstacleGrid
        for i in range(height):
            for j in range(width):
                if (i, j) in obstacles:
                    self.grid[i, j] = State((i, j), True, None, None)
                else:
                    self.grid[i, j] = State((i, j), False, None, None)
        self.costGrid1 = np.random.randint(1, 10, (height, width))
        self.costGrid2 = np.random.randint(1, 10, (height, width))
        self.hGrid1 = self.heuristicGrid(self.costGrid1)
        self.hGrid2 = self.heuristicGrid(self.costGrid2)
        for i in range(height):
            for j in range(width):
                if self.grid[i, j].isObs:
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
                    or self.grid[i, j].isObs):
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
                    or self.grid[i, j].isObs):
                    continue
                else:
                    neighbors.append((i, j))
        return neighbors
    
    def heuristicGrid(self, costGrid):
        grid = np.empty((self.height, self.width), dtype=object)
        unvisited = PriorityQueue()
        for i in range(self.height):
            for j in range(self.width):
                if self.grid[i, j].isObs:
                    grid[i, j] = DijkstraNode((i, j), INF, True)
                else:
                    grid[i, j] = DijkstraNode((i, j), INF, False)
                    unvisited.push(grid[i, j])
        
        curNode = grid[self.posGoal]
        curNode.dist = 0
        unvisited.updateOrder()
        while not unvisited.empty():
            curPos = curNode.pos
            newDist = curNode.dist + costGrid[curPos]
            neighborsPos = self.getNeighborsPos(curPos)
            for neighborPos in neighborsPos:
                neighbor = grid[neighborPos]
                if neighbor.visited:
                    continue
                if newDist < neighbor.dist:
                    neighbor.dist = newDist
            curNode.visited = True
            unvisited.updateOrder()
            curNode = unvisited.pop()
        
        distGrid = np.zeros((self.height, self.width))
        for i in range(self.height):
            for j in range(self.width):
                distGrid[i, j] = grid[i, j].dist

        return distGrid

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
        newHeap = self.heap + list
        heapq.heapify(newHeap)
        self.heap = newHeap

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

def filterOpen(vector, openList):
    removeList = []
    for x in openList:
        s = x.state
        if dominates(vector, x.f):
            removeList.append(x)
            s.gOp.remove(x)
    for x in removeList:
        openList.remove(x)
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

def publishPath(path, obstacleGrid):
    grid = copy.deepcopy(obstacleGrid)

    for pos in path:
        grid[pos[0]][pos[1]] = 8
    for row in grid:
        print(row)
    print("")

def publishSolutions(sols, obstacleGrid):
    paths = []
    for sol in sols:
        nTmp = sol
        path = []
        while nTmp != None:
            path.append(nTmp.state.pos)
            nTmp = nTmp.parent
        path.reverse()
        publishPath(path, obstacleGrid)
        paths.append(path)
    return paths

def namoa(problem):
    sols = []
    open = PriorityQueue()

    sStart = problem.getStart()
    sGoal = problem.getGoal()

    nStart = Node(sStart, np.array([0, 0]), None)

    sStart.gOp.add(nStart)
    open.push(nStart)
    
    while not open.empty():
        x = open.pop()
        s = x.state

        s.gOp.remove(x)
        s.gCl.add(x)
        if s == sGoal:
            sols.append(x)
            filterOpen(x.g, open.heap)
            filterSet(x.g, sols)
            continue
        for t in problem.getSuccessors(s):
            gy = x.g + t.cost
            y = Node(t, gy, x)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.f):
                continue

            filterGOp(y.g, t.gOp, open.heap)
            filterSet(y.g, t.gCl)

            t.gOp.add(y)
            open.push(y)

    return sols
  
def main():
    # obstacleGrid = [[0, 0, 0, 0, 0, 0],
    #                 [1, 0, 1, 1, 1, 0],
    #                 [1, 0, 1, 1, 1, 0],
    #                 [0, 0, 1, 0, 0, 0],
    #                 [0, 1, 1, 0, 1, 0],
    #                 [0, 0, 0, 0, 1, 0],
    #                 [1, 1, 1, 1, 1, 0]]

    obstacleGrid = [[0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    obstacles = []
    for i, row in enumerate(obstacleGrid):
        for j in range(len(row)):
            if obstacleGrid[i][j] == 1:
                obstacles.append((i, j))

    problem = GridProblem(len(obstacleGrid), len(obstacleGrid[0]), 
                         (0, 0), (len(obstacleGrid)-1, len(obstacleGrid[0])-1), 
                         obstacles, obstacleGrid)
    
    sols = namoa(problem)

    print("Pareto Optimal Set")
    publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()
