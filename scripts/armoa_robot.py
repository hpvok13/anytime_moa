import numpy as np
from helper import *
import timeit

def ImprovePath(problem, sols, open, incons, epsilon):
    sGoal = problem.getGoal()
    stateSet = set()
    
    while not open.empty():
        x = open.pop()
        s = x.state

        if s in stateSet:
            s = 
        

        if setDominates(sols, x.f):
            s.gOp.remove(x)
            incons.append(x)
            continue

        s.gOp.remove(x)
        s.gCl.add(x)
        if s == sGoal:
            sols.append(x)
            filterOpenArmoa(x.g, open.heap) #might be able to remove
            filterSet(x.g, sols)
            continue
        for t in problem.getSuccessors(s):
            gy = x.g + t.cost
            y = ArmoaNode(t, gy, x, epsilon)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.fUnweighted):####
                continue
            
            filterGOp(y.g, t.gOp, open.heap)
            filterSet(y.g, t.gCl)

            t.gOp.add(y)
            open.push(y)

def armoaIncon(problem, doPrint):
    sols = []
    incons = []
    open = PriorityQueue()

    # epsPlot = []
    # numSolsPlot = []
    # compTimePlot = []

    sStart = problem.getStart()

    epsilon = 2

    nStart = ArmoaNode(sStart, np.array([0, 0]), None, epsilon)

    sStart.gOp.add(nStart)
    open.push(nStart)

    # startTime = timeit.default_timer()
    ImprovePath(problem, sols, open, incons, epsilon)
    # duration = timeit.default_timer()-startTime

    # epsPlot.append(epsilon)
    # numSolsPlot.append(len(sols))
    # compTimePlot.append(duration)
    
    # print("Epsilon " + str(epsilon) + ":")
    # print("----------------------------------------------------------------")
    # print(duration)
    # publishSolutions(sols, problem.obstacleGrid, doPrint)

    # publishPathCosts(sols)

    while(shouldIterate(sols, incons)):
        epsilon -= 0.05
        if epsilon < 1:
            epsilon = 1

        open.heap = incons
        incons = []
        
        updateFOpenIncon(open.heap, epsilon)
        
        # print("\nEpsilon " + str(epsilon) + ":")
        # print("----------------------------------------------------------------")
        
        # startTime = timeit.default_timer()
        ImprovePath(problem, sols, open, incons, epsilon)
        # duration = timeit.default_timer()-startTime

        # epsPlot.append(epsilon)
        # numSolsPlot.append(len(sols))
        # compTimePlot.append(duration)
        
        # print(duration)
        # publishSolutions(sols, problem.obstacleGrid, doPrint)

        # publishPathCosts(sols)
    
    # plt.title("Pareto Cost Front")
    # plt.xlabel("C_0()")
    # plt.ylabel("C_1()")
    # plt.show()

    # plt.plot(epsPlot, numSolsPlot)
    # plt.title("Number of Solutions in Pareto Suboptimal Set vs Epsilon")
    # plt.xlabel("Epsilon")
    # plt.ylabel("Number of Solutions")
    # plt.show()

    # plt.plot(epsPlot, compTimePlot)
    # plt.title("Computation Time in Pareto Suboptimal Set vs Epsilon")
    # plt.xlabel("Epsilon")
    # plt.ylabel("Computation Time")
    # plt.show()

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
    sols = armoaIncon(problem)
    duration = timeit.default_timer() - startTime

    print("\nPareto Optimal Set")
    print(duration)
    # print("----------------------------------------------------------------")
    # publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()
