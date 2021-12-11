import numpy as np
import timeit
from helper import *
from armoa import armoa
from namoa import namoa
from armoa_inconsistent import armoaIncon


doPrint = True

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

def createCostGrids(height, width, fname1, fname2):
    costGrid1 = np.random.randint(1, 10, (height, width))
    costGrid2 = np.random.randint(1, 10, (height, width))
    np.savetxt(fname1, costGrid1, delimiter=',')
    np.savetxt(fname2, costGrid2, delimiter=',')

def main():
    obstacleGridStr = np.genfromtxt("../maps/maze_32_32_2.map", delimiter=1, skip_header=4, dtype=str)

    obstacleGrid = np.zeros(obstacleGridStr.shape)

    for i in range(obstacleGridStr.shape[0]):
        for j in range(obstacleGridStr.shape[1]):
            if obstacleGridStr[i, j] == '@':
                obstacleGrid[i, j] = 1

    # obstacleGrid = np.array([[0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                         [0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                         [0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                         [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                         [0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    # costGrid1 = np.loadtxt("../costGrids/maze_32_32_2_cost_grid_1.csv", delimiter=',')
    # costGrid2 = np.loadtxt("../costGrids/maze_32_32_2_cost_grid_2.csv", delimiter=',')
    costGrid1 = None
    costGrid2 = None

    problem = GridProblem((1, 1), (obstacleGrid.shape[0]-1, obstacleGrid.shape[1]-1), obstacleGrid, costGrid1, costGrid2)


    print("ARMOA* with Inconsistent Set")
    startTime = timeit.default_timer()
    solsArmoaIncon = armoaIncon(problem, doPrint)
    duration = timeit.default_timer() - startTime

    print("\nTotal")
    print(duration)


    print("\n\nARMOA*")
    startTime = timeit.default_timer()
    solsArmoa = armoa(problem, doPrint)
    duration = timeit.default_timer() - startTime

    print("\nTotal")
    print(duration)

    print("\n\nNAMOA*")
    startTime = timeit.default_timer()
    solsNamoa = namoa(problem)
    duration = timeit.default_timer() - startTime

    print(duration)

    # print("\n\n\nARMOA* solution set:")
    # publishSolutions(solsArmoa, obstacleGrid, doPrint)

    # print("\n\n\nARMOA* solution set:")
    # publishSolutions(solsArmoa, obstacleGrid, doPrint)

    # print("\n\nNAMOA* solution set:")
    # publishSolutions(solsNamoa, obstacleGrid, doPrint)

    if verifyPaths(solsArmoaIncon, solsNamoa):
        print("ARMOA* with Inconsistent Set solution set matches NAMOA*")

    if verifyPaths(solsArmoa, solsNamoa):
        print("ARMOA* solution set matches NAMOA*")

if __name__ == '__main__':
    # grid = np.genfromtxt("../maps/maze_32_32_2.map", delimiter=1, skip_header=4, dtype=str)

    # createCostGrids(grid.shape[0], grid.shape[1], '../costGrids/maze_32_32_2_cost_grid_1.csv', '../costGrids/maze_32_32_2_cost_grid_2.csv')

    main()