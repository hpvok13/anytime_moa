import numpy as np
import timeit
from helper import *
from armoa import armoa
from namoa import namoa
from armoa_inconsistent import armoaIncon


doPrint = False

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

    armoaTimes = []
    namoaTimes = []

    for _ in range(50):
        problem = GridProblem((1, 1), (obstacleGrid.shape[0]-1, obstacleGrid.shape[1]-1), obstacleGrid, costGrid1, costGrid2)


        print("ARMOA* with Inconsistent Set")
        startTime = timeit.default_timer()
        solsArmoaIncon = armoaIncon(problem, doPrint)
        duration = timeit.default_timer() - startTime

        print("\nTotal")
        print(duration)

        armoaTimes.append(duration)

        print("\n\nNAMOA*")
        startTime = timeit.default_timer()
        solsNamoa = namoa(problem)
        duration = timeit.default_timer() - startTime

        print(duration)

        namoaTimes.append(duration)

        if not verifyPaths(solsArmoaIncon, solsNamoa):
            print("Incorrect paths")
            # print("ARMOA* with Inconsistent Set solution set matches NAMOA*")
        
    print("ARMOA* Times:")
    print(armoaTimes)
    print("NAMOA* Times:")
    print(namoaTimes)

    print("Avg ARMOA* Times")
    print(np.mean(armoaTimes))
    print("Avg NAMOA* Times")
    print(np.mean(namoaTimes))
    

def compTimes():
    namoaTimes = [40.19]
    armoaTimes = [83.28]

if __name__ == '__main__':
    # grid = np.genfromtxt("../maps/maze_32_32_2.map", delimiter=1, skip_header=4, dtype=str)

    # createCostGrids(grid.shape[0], grid.shape[1], '../costGrids/maze_32_32_2_cost_grid_1.csv', '../costGrids/maze_32_32_2_cost_grid_2.csv')

    main()