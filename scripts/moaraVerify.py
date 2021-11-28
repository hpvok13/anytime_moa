import numpy as np
import timeit
from helper import *
from moara import moara
from namoa import namoa


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

def main():
    obstacleGridStr = np.genfromtxt("../maps/Berlin_1_256.map", delimiter=1, skip_header=4, dtype=str)

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

    problem = GridProblem((0, 0), (obstacleGrid.shape[0]-1, obstacleGrid.shape[1]-1), obstacleGrid)


    print("MOARA*")
    startTime = timeit.default_timer()
    solsMoara = moara(problem, doPrint)
    duration = timeit.default_timer() - startTime

    print("\nTotal")
    print(duration)

    print("\n\nNAMOA*")
    startTime = timeit.default_timer()
    solsNamoa = namoa(problem)
    duration = timeit.default_timer() - startTime

    print(duration)

    print("\n\n\nMOARA* solution set:")
    publishSolutions(solsMoara, obstacleGrid, doPrint)

    print("\n\nNAMOA* solution set:")
    publishSolutions(solsNamoa, obstacleGrid, doPrint)

    if verifyPaths(solsMoara, solsNamoa):
        print("MOARA* solution set matches NAMOA*")

if __name__ == '__main__':
    main()