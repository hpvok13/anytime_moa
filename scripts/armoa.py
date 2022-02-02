import numpy as np
from helper import *
import timeit

def ImprovePath(problem, sols, open, epsilon):
    sGoal = problem.getGoal()
    
    while not shouldTerminate(sols, open.heap):
        x = open.pop()
        s = x.state

        s.gOp.remove(x)
        s.gCl.add(x)
        if s == sGoal:
            sols.append(x)
            filterOpenArmoa(x.g, open.heap)
            filterSet(x.g, sols)
            continue
        for t in problem.getSuccessors(s):
            gy = x.g + t.cost
            y = ArmoaNode(t, gy, x, epsilon)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.fUnweighted):
                continue

            filterGOp(y.g, t.gOp, open.heap)
            filterSet(y.g, t.gCl)

            t.gOp.add(y)
            open.push(y)

def armoa(problem, doPrint):
    sols = []
    open = PriorityQueue()

    sStart = problem.getStart()

    epsilon = 2

    nStart = ArmoaNode(sStart, np.array([0, 0]), None, epsilon)

    sStart.gOp.add(nStart)
    open.push(nStart)

    startTime = timeit.default_timer()
    ImprovePath(problem, sols, open, epsilon)
    duration = timeit.default_timer()-startTime
    
    print("Epsilon " + str(epsilon) + ":")
    print("----------------------------------------------------------------")
    print(duration)
    publishSolutions(sols, problem.obstacleGrid, doPrint)
    publishPathCosts(sols)

    while(shouldIterate(sols, open.heap)):
        epsilon -= 0.05
        if epsilon < 1:
            epsilon = 1
        
        updateFOpen(open.heap, epsilon)
        
        print("\nEpsilon " + str(epsilon) + ":")
        print("----------------------------------------------------------------")
        
        startTime = timeit.default_timer()
        ImprovePath(problem, sols, open, epsilon)
        duration = timeit.default_timer()-startTime
        
        print(duration)
        publishSolutions(sols, problem.obstacleGrid, doPrint)

        publishPathCosts(sols)
    
    plt.title("Pareto Cost Front")
    plt.xlabel("C_0(x)")
    plt.ylabel("C_1(x)")
    plt.show()
    
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
    sols = armoa(problem)
    duration = timeit.default_timer() - startTime

    print("\nPareto Optimal Set")
    print(duration)
    # print("----------------------------------------------------------------")
    # publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()
