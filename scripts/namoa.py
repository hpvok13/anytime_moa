import numpy as np
import timeit
from helper import *

INF = 2**32 - 1

def namoa(problem):
    sols = []
    open = PriorityQueue()

    sStart = problem.getStart()
    sGoal = problem.getGoal()

    nStart = NamoaNode(sStart, np.array([0, 0]), None)

    sStart.gOp.add(nStart)
    open.push(nStart)
    
    while not open.empty():
        x = open.pop()
        s = x.state

        s.gOp.remove(x)
        s.gCl.add(x)
        if s == sGoal:
            sols.append(x)
            filterOpenNamoa(x.g, open.heap)
            # filterSet(x.g, sols)
            continue
        for t in problem.getSuccessors(s):
            gy = x.g + t.cost
            y = NamoaNode(t, gy, x)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.f):
                continue

            filterGOp(y.g, t.gOp, open.heap)
            filterSet(y.g, t.gCl)

            t.gOp.add(y)
            open.push(y)

    return sols
  
def main():
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
    
    startTime = timeit.default_timer()
    sols = namoa(problem)
    duration = timeit.default_timer() - startTime

    print(duration)
    # print("Pareto Optimal Set")
    # publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()
