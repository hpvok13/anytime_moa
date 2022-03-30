import numpy as np
from mobile_robot import *
from helper_mobile import *
import timeit

def ImprovePath(problem, sols, open, incons, epsilon):
    sGoal = problem.getGoalState()
    while not open.empty():
        x = open.pop()
        s = x.state

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
        sPose = s.pose
        for t in problem.getSuccessors(s):
            tPose = t.pose
            distCost = np.linalg.norm(np.array([tPose[0] - sPose[0], 
                                                tPose[1] - sPose[1], 
                                                subtractAngles(tPose[2], sPose[2])]))
            safeCost = problem.safeCost[tuple(t.idx[:2])]

            gy = x.g + np.array([distCost, safeCost])
            y = ArmoaNode(t, gy, x, epsilon)
            if setDominates(t.gOp, y.g) or setDominates(t.gCl, y.g) or setDominates(sols, y.fUnweighted):####
                continue
            
            filterGOp(y.g, t.gOp, open.heap)
            filterSet(y.g, t.gCl)

            t.gOp.add(y)
            open.push(y)

def armoaRobot(problem, epsilonStart):
    sols = []
    incons = []
    open = PriorityQueue()

    # epsPlot = []
    # numSolsPlot = []
    # compTimePlot = []

    sStart = problem.getStartState()

    epsilon = epsilonStart

    nStart = ArmoaNode(sStart, np.array([0, 0]), None, epsilon)

    sStart.gOp.add(nStart)
    open.push(nStart)

    # startTime = timeit.default_timer()
    ImprovePath(problem, sols, open, incons, epsilon)
    # duration = timeit.default_timer()-startTime

    # epsPlot.append(epsilon)
    # numSolsPlot.append(len(sols))
    # compTimePlot.append(duration)
    
    print("\nEpsilon {:.2f}:".format(epsilon))
    print("----------------------------------------------------------------")
    # print(str(len(sols)) + " Solutions")
    # print(duration)
    publishSolutions(sols, problem)

    # publishPathCosts(sols)

    while(shouldIterate(sols, incons)):
        epsilon -= 0.01
        if epsilon < 1:
            epsilon = 1

        open.heap = incons
        incons = []
        
        updateFOpenIncon(open.heap, epsilon)
        
        print("\nEpsilon {:.2f}:".format(epsilon))
        print("----------------------------------------------------------------")
        
        # startTime = timeit.default_timer()
        ImprovePath(problem, sols, open, incons, epsilon)
        # duration = timeit.default_timer()-startTime

        # epsPlot.append(epsilon)
        # numSolsPlot.append(len(sols))
        # compTimePlot.append(duration)
        
        # print(duration)
        if len(sols) > 1:
            publishSolutions(sols, problem)
        else:
            print(str(len(sols)) + " Solutions")
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
    poseStart = np.array((0, 0, 0))
    poseGoal = np.array((1.5, 2.5, -math.pi))
    height = 2
    width = 3
    numThetas = 60
    obstacles = [Obstacle((1, 1.5), 0.5),
                 Obstacle((0.5, 2), 0.3)]
    statesPerMeter = 20
    robot = ShipBot(0.2, 0.1, 0.05)
    maxTurnTicks = 1
    calculateHeuristic = False
    epsilonStart = 2

    problem = ShipBotProblem(poseStart, poseGoal, height, width, numThetas, obstacles, robot, statesPerMeter, maxTurnTicks, calculateHeuristic)
    print("Problem Created")
    # startTime = timeit.default_timer()
    sols = armoaRobot(problem, epsilonStart)
    # duration = timeit.default_timer() - startTime

    # print("\nPareto Optimal Set")
    # print(duration)
    # print("----------------------------------------------------------------")
    # publishSolutions(sols, problem.obstacleGrid)

if __name__ == "__main__":
    main()