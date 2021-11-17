import numpy as np
import timeit
from helper import *
from moara import moara
from namoa import namoa

def verifyPaths(sols1, sols2):
    if len(sols1) != len(sols2):
        print("Pareto optimal solution sets have different length")
        return False
    
    if sols1 != sols2:
        print("Solution sets do not equal")
        return False
        
    #     for sol2 in sols2:
    #         for i, row1 in enumerate(sol1):
    #             if row1 != sol2[i]:
    #                 break
    #         else:
    #             break
    #     else:
    #         print("Solutions don't match")
    #         return False
    return True

obstacleGridStr = np.genfromtxt("../maps/Berlin_1_256.map", delimiter=1, skip_header=4, dtype=str)

obstacleGrid = np.zeros(obstacleGridStr.shape)

obstacles = []
for i in range(obstacleGridStr.shape[0]):
    for j in range(obstacleGridStr.shape[1]):
        if obstacleGridStr[i, j] == '@':
            obstacleGrid[i, j] = 1
            obstacles.append((i, j))

# obstacleGrid = [[0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

# obstacles = []
# for i, row in enumerate(obstacleGrid):
#     for j in range(len(row)):
#         if obstacleGrid[i][j] == 1:
#             obstacles.append((i, j))

problem = GridProblem((0, 0), (obstacleGrid.shape[0]-1, obstacleGrid.shape[1]-1), obstacleGrid)


print("MOARA*")
startTime = timeit.default_timer()
solsMoara = moara(problem)
duration = timeit.default_timer() - startTime

print("\nTotal")
print(duration)

print("\n\nNAMOA*")
startTime = timeit.default_timer()
solsNamoa = namoa(problem)
duration = timeit.default_timer() - startTime

print(duration)

print("\n\n\nMOARA* solution set:")
pathsMoara = publishSolutions(solsMoara, obstacleGrid, True)

print("\n\nNAMOA* solution set:")
pathsNamoa = publishSolutions(solsNamoa, obstacleGrid, True)


print(verifyPaths(pathsMoara, pathsNamoa))
