import heapq
import numpy as np
import copy

INF = 2**32 - 1

class node:
    def __init__(self, state, g, parent):
        self.state = state
        self.g = g
        self.f = (g[0] + state.h[0], g[1] + state.h[1])
        self.parent = parent

    def __lt__(self, other):
        return lexiLT(self.f, other.f)

    def dominates(self, other):
        return dominates(self.f, other.f)

class dijkstraNode:
    def __init__(self, pos, dist, visited):
        self.pos = pos
        self.dist = dist
        self.visited = visited
    
    def __lt__(self, other):
        return self.dist < other.dist

class state:
    def __init__(self, pos, isObs, h, cost):
        self.pos = pos
        self.isObs = isObs
        self.g2Min = INF
        self.h = h
        self.cost = cost

class gridProblem:
    def __init__(self, h, w, posStart, posGoal, obstacles, obstacleGrid):
        self.h = h
        self.w = w
        self.grid = np.empty((h, w), dtype=object)
        self.posStart = posStart
        self.posGoal = posGoal
        self.obstacles = obstacles
        self.obstacleGrid = obstacleGrid
        for i in range(h):
            for j in range(w):
                if (i, j) in obstacles:
                    self.grid[i, j] = state((i, j), True, None, None)
                else:
                    self.grid[i, j] = state((i, j), False, None, None)
        self.costGrid1 = np.random.randint(1, 10, (h, w))
        self.costGrid2 = np.random.randint(1, 10, (h, w))
        self.hGrid1 = self.heuristicGrid(self.costGrid1)
        self.hGrid2 = self.heuristicGrid(self.costGrid2)
        for i in range(h):
            for j in range(w):
                if self.grid[i, j].isObs:
                    continue
                else:
                    self.grid[i, j].h = (self.hGrid1[i, j], self.hGrid2[i, j])
                    self.grid[i, j].cost = (self.costGrid1[i, j], self.costGrid2[i, j])
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
                    or i < 0 or i >= self.h
                    or j < 0 or j >= self.w
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
                    or i < 0 or i >= self.h
                    or j < 0 or j >= self.w
                    or self.grid[i, j].isObs):
                    continue
                else:
                    neighbors.append((i, j))
        return neighbors
    
    def heuristicGrid(self, costGrid):
        grid = np.empty((self.h, self.w), dtype=object)
        unvisited = priorityQueue()
        for i in range(self.h):
            for j in range(self.w):
                if self.grid[i, j].isObs:
                    grid[i, j] = dijkstraNode((i, j), INF, True)
                else:
                    grid[i, j] = dijkstraNode((i, j), INF, False)
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
        
        distGrid = np.zeros((self.h, self.w))
        for i in range(self.h):
            for j in range(self.w):
                distGrid[i, j] = grid[i, j].dist

        return distGrid

class priorityQueue:
    def __init__(self, list=None):
        if list == None:
            self.heap = []
        else:
            self.heap = list
            heapq.heapify(self.heap)

    def empty(self):
        return len(self.heap) == 0

    def push(self, n):
        heapq.heappush(self.heap, n)

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

def publishSolution(path, expanded, obstacleGrid):
    grid = copy.deepcopy(obstacleGrid)

    for pos in expanded:
        grid[pos[0]][pos[1]] = 7
    for pos in path:
        grid[pos[0]][pos[1]] = 8
    for row in grid:
        print(row)

    print("Path:")
    print(path)

def publishExpanded(expanded):
    print("Expanded States:")
    print(expanded)

def publishProgress(expanded, obstacleGrid):
    grid = copy.deepcopy(obstacleGrid)

    for pos in expanded:
        grid[pos[0]][pos[1]] = 7

    print("Iter 1:")
    for row in grid:
        print(row)

def namoa(problem):
    sols = []
    expanded = []

    sStart = problem.getStart()
    sGoal = problem.getGoal()
    nStart = node(sStart, (0, 0), None)
    
    open = priorityQueue()
    open.push(nStart)
    while not open.empty():
        #publishProgress(expanded, problem.obstacleGrid)
        x = open.pop()
        s = x.state
        expanded.append(s.pos)
        if (x.g[1] >= s.g2Min
            or x.f[1] >= sGoal.g2Min):
            continue
        s.g2Min = x.g[1]
        if s == sGoal:
            sols.append(x)
            continue
        for t in problem.getSuccessors(s):
            gy = (x.g[0] + t.cost[0], x.g[1] + t.cost[1])
            y = node(t, gy, x)
            if (y.g[1] >= t.g2Min
            or y.f[1] >= sGoal.g2Min):
                continue
            open.push(y)

    paths = []
    for sol in sols:
        nTmp = sol
        path = []
        while nTmp != None:
            path.insert(0, nTmp.state.pos)
            nTmp = nTmp.parent
        paths.append(path)
    
    return paths, expanded
  
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

    problem = gridProblem(len(obstacleGrid), len(obstacleGrid[0]), 
                         (0, 0), (len(obstacleGrid)-1, len(obstacleGrid[0])-1), 
                         obstacles, obstacleGrid)
    
    paths, expanded = namoa(problem)

    for path in paths:
        publishSolution(path, expanded, obstacleGrid)

    #publishExpanded(expanded)


if __name__ == "__main__":
    main()
