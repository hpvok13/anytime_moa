import numpy as np
from robot_arm import *
from helper_robot import *
import timeit

def ImprovePath(problem, sols, open, incons, epsilon):
    cellGoal = problem.getGoalCell()
    n = problem.robot.n
    
    while not open.empty():
        x = open.pop()
        s = x.state
        c = s.cell

        if setDominates(sols, x.f):
            s.gOp.remove(x)
            incons.append(x)
            continue

        s.gOp.remove(x)
        s.gCl.add(x)
        if c == cellGoal:
            sols.append(x)
            filterOpenArmoa(x.g, open.heap) #might be able to remove
            filterSet(x.g, sols)
            continue
        for t in problem.getSuccessors(s):
            distCost = 0.25
            gy = x.g + np.array([distCost, 0])
            y = ArmoaNode(t, gy, x, epsilon)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.fUnweighted):####
                continue
            
            filterGOp(y.g, t.gOp, open.heap)
            filterSet(y.g, t.gCl)

            t.gOp.add(y)
            open.push(y)

def armoaRobot(problem, doPrint):
    sols = []
    incons = []
    open = PriorityQueue()

    # epsPlot = []
    # numSolsPlot = []
    # compTimePlot = []

    sStart = problem.getStartState()

    epsilon = 10

    nStart = ArmoaNode(sStart, np.array([0, 0]), None, epsilon)

    sStart.gOp.add(nStart)
    open.push(nStart)

    # startTime = timeit.default_timer()
    ImprovePath(problem, sols, open, incons, epsilon)
    # duration = timeit.default_timer()-startTime

    # epsPlot.append(epsilon)
    # numSolsPlot.append(len(sols))
    # compTimePlot.append(duration)
    
    print("Epsilon " + str(epsilon) + ":")
    print("----------------------------------------------------------------")
    # print(duration)
    publishSolutions(sols, problem, doPrint)

    # publishPathCosts(sols)

    while(shouldIterate(sols, incons)):
        epsilon -= 0.05
        if epsilon < 1:
            epsilon = 1

        open.heap = incons
        incons = []
        
        updateFOpenIncon(open.heap, epsilon)
        
        print("\nEpsilon " + str(epsilon) + ":")
        print("----------------------------------------------------------------")
        
        # startTime = timeit.default_timer()
        ImprovePath(problem, sols, open, incons, epsilon)
        # duration = timeit.default_timer()-startTime

        # epsPlot.append(epsilon)
        # numSolsPlot.append(len(sols))
        # compTimePlot.append(duration)
        
        # print(duration)
        publishSolutions(sols, problem, doPrint)

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
    obstacleGrid = np.zeros((24, 48))
    cellSize = 0.25
    robot = Robot([1,1,1,1,1,1], cellSize)
    thetas = [math.pi/4,0,0,0,0,0]
    posGoal = np.array((-3, 2))
    
    problem = RobotProblem(thetas, posGoal, obstacleGrid, robot, cellSize)
    
    # startTime = timeit.default_timer()
    sols = armoaRobot(problem, False)
    # duration = timeit.default_timer() - startTime

    # print("\nPareto Optimal Set")
    # print(duration)
    # print("----------------------------------------------------------------")
    # publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()
