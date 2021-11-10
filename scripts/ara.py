import heapq
import numpy as np
import copy

INF = 2**32 - 1

class state:
    def __init__(self, pos, isObs, g, posGoal):
        self.pos = pos
        self.isObs = isObs
        self.g = g
        self.h = max(abs(pos[0] - posGoal[0]), abs(pos[1] - posGoal[1]))
        self.parent = None

    def __lt__(self, other):
        if type(other) is type(self):
            if self.fValue(1) < other.fValue(1):
                return True
        else:
            return False

    def gValue(self):
        return self.g

    def hValue(self):
        return self.h

    def fValue(self, eps):
        return self.g + eps*self.h

    def updateGValue(self, g):
        self.g = g
    

class gridProblem:
    def __init__(self, h, w, posStart, posGoal, obstacles):
        self.h = h
        self.w = w
        self.grid = np.empty((h, w), dtype=object)
        self.posStart = posStart
        self.posGoal = posGoal
        self.obstacles = obstacles
        for i in range(h):
            for j in range(w):
                if (i, j) in obstacles:
                    self.grid[i, j] = state((i, j), True, -1, posGoal)
                elif (i, j) == posStart:
                    self.grid[i, j] = state((i, j), False, 0, posGoal)
                else:
                    self.grid[i, j] = state((i, j), False, INF, posGoal)
    
    def getPosStart(self):
        return self.posStart
    
    def getPosGoal(self):
        return self.posGoal

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

class priorityQueue:
    def __init__(self, list=None):
        if list == None:
            self.heap = []
        else:
            self.heap = list
            heapq.heapify(self.heap)

    def push(self, s, eps):
        heapq.heappush(self.heap, (s.fValue(eps), s))

    def pop(self):
        return heapq.heappop(self.heap)[1]

    def pushUpdatePriority(self, s, eps):
        for i, (p, x) in enumerate(self.heap):
            if x == s:
                if p < s.fValue(eps):
                    break
                del self.heap[i]
                self.heap.append((s.fValue(eps), s))
                heapq.heapify(self.heap)
                break
        else:
            self.push(s, eps)
                    
    def peek(self):
        return self.heap[0][1]

    def mergeUpdate(self, list, eps):
        newHeap = []
        for (_, s) in self.heap:
            newHeap.append((s.fValue(eps), s))
        
        for s in list:
            newHeap.append((s.fValue(eps), s))

        heapq.heapify(newHeap)
        
        self.heap = newHeap

    def minF(self, eps):
        minF = INF + 1
        for _, s in self.heap:
            if s.fValue(eps) < minF:
                minF = s.fValue(eps)
        return minF


def minFInSet(set, eps):
    minF = INF + 1
    for s in set:
        if s.fValue(eps) < minF:
            minF = s.fValue(eps)
    return minF

def publishSolution(epsPrime, path, closed, obstacleGrid):
    grid = copy.deepcopy(obstacleGrid)
    print("Epsilon':")
    print(epsPrime)

    for s in closed:
        grid[s.pos[0]][s.pos[1]] = 7
    for pos in path:
        grid[pos[0]][pos[1]] = 8
    for row in grid:
        print(row)

    print("Path:")
    print(path)

    expanded = []
    for s in closed:
        expanded.append(s.pos)
    print("Expanded States:")
    print(expanded)

def improvePath(problem, open, closed, incons, seen, eps):
    sStart = problem.getStart()
    sGoal = problem.getGoal()
    while sGoal.fValue(eps) > open.peek().fValue(eps):
        s = open.pop()
        closed.append(s)
        for sPrime in problem.getSuccessors(s):
            if sPrime not in seen:
                sPrime.updateGValue(INF)
                seen.add(sPrime)
            if sPrime.gValue() > s.gValue() + 1:
                sPrime.updateGValue(s.gValue() + 1)
                if sPrime != sStart:
                    sPrime.parent = s #fix
                if sPrime not in closed:
                    open.pushUpdatePriority(sPrime, eps)
                else:
                    incons.add(sPrime)
    
    sTmp = problem.getGoal()
    path = []
    while sTmp != None:
        path.insert(0, sTmp.pos)
        sTmp = sTmp.parent
    
    return path

def main():
    obstacleGrid = [[0, 0, 0, 0, 0, 0],
                    [1, 0, 1, 1, 1, 0],
                    [1, 0, 1, 1, 1, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 1, 1, 0, 1, 0],
                    [0, 0, 0, 0, 1, 0],
                    [1, 1, 1, 1, 1, 0]]

    obstacles = []
    for i, row in enumerate(obstacleGrid):
        for j in range(len(row)):
            if obstacleGrid[i][j] == 1:
                obstacles.append((i, j))

    problem = gridProblem(7, 6, (0, 0), (6, 5), obstacles)

    sStart = problem.getStart()

    eps = 2.5

    closed = []
    seen = set()
    incons = set()
    open = priorityQueue()
    open.push(sStart, eps)

    path = improvePath(problem, open, closed, incons, seen, eps)

    epsPrime = min(eps, problem.getGoal().gValue()/min(open.minF(1), minFInSet(incons, 1)))

    publishSolution(epsPrime, path, closed, obstacleGrid)

    while epsPrime > 1:
        eps -= 1
        if eps < 1:
            eps = 1
        
        open.mergeUpdate(list(incons), eps)

        closed = []

        path = improvePath(problem, open, closed, incons, seen, eps)

        epsPrime = min(eps, problem.getGoal().gValue()/min(open.peek().fValue(1), minFInSet(incons, 1)))

        publishSolution(epsPrime, path, closed, obstacleGrid)

if __name__ == "__main__":
    main()
