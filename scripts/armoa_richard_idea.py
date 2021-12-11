import numpy as np
from helper import *

INF = 2**32 - 1

def ImprovePath(problem, sols, open, incons, epsilon):
    sGoal = problem.getGoal()
    
    while not open.empty():
        x = open.pop()
        s = x.state
        
        s.gOp.remove(x)
        s.gCl.add(x)

        if s == sGoal:
            sols.append(x)
            filterOpenMoara(x.g, open.heap)
            filterSet(x.g, sols)
            continue
        for t in problem.getSuccessors(s):
            gy = x.g + t.cost
            y = MoaraNode(t, gy, x, epsilon)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.fUnweighted):
                continue

            filterGOp(y.g, t.gOp, open.heap)
            if filterSet(y.g, t.gCl):
                incons.append(y)
                continue

            t.gOp.add(y)
            open.push(y)

def moara(problem):
    sols = []
    open = PriorityQueue()
    incons = []

    sStart = problem.getStart()

    epsilon = 5

    nStart = MoaraNode(sStart, np.array([0, 0]), None, epsilon)

    sStart.gOp.add(nStart)
    open.push(nStart)

    print("\n\nEpsilon " + str(epsilon) + ":")
    print("----------------------------------------------------------------")

    ImprovePath(problem, sols, open, incons, epsilon)
    
    publishSolutions(sols, problem.obstacleGrid)

    while(shouldIterate(sols, incons)):
        epsilon -= 1
        if epsilon < 1:
            epsilon = 1
        
        open.heap = incons
        incons = []

        updateFOpen(open.heap, epsilon)

        print("\n\nEpsilon " + str(epsilon) + ":")
        print("----------------------------------------------------------------")
        
        ImprovePath(problem, sols, open, incons, epsilon)

        publishSolutions(sols, problem.obstacleGrid)
    
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
    
    sols = moara(problem)

    print("Pareto Optimal Set")
    publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()
