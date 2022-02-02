from os import link
import matplotlib.pyplot as plt
import math
import numpy as np
from sklearn import neighbors

cellSize = 0.25

# class Action:
#     def __init__(self, thetas, jointPos):
#         self.thetas = thetas
#         self.jointPos = jointPos

class Robot:
    def __init__(self, linkLengths):
        self.linkLengths = np.array(linkLengths, dtype='float')
        self.n = len(self.linkLengths)

        self.moveAngles = np.zeros((self.n, 1))
        for i in range(self.n):
            self.moveAngles[i] = gridSize/linkLengths[i]

    def visualize(self, thetas=None, jointPos=None):
        if thetas != None:
            jointPos = self.fk(thetas)
        plt.figure()
        for i in range(self.n):
            lineX = jointPos[i:i+2, 0]
            lineY = jointPos[i:i+2, 1]
            plt.plot(lineX, lineY)

        bound = sum(self.linkLengths)*1.25
        plt.axis('scaled')
        plt.xlim([-bound,bound])
        plt.ylim([0,bound])
        plt.show()
    
    def fk(self, thetas):
        cumThetas = np.cumsum(thetas)
        jointPos = np.zeros((self.n+1, 2))
        for i in range(self.n):
            ll = self.linkLengths[i]
            cumTheta = cumThetas[i]
            jointPos[i+1,0] = jointPos[i,0] + ll*math.cos(cumTheta)
            jointPos[i+1,1] = jointPos[i,1] + ll*math.sin(cumTheta)
        return jointPos

    def getNeighbors(self, thetas):
        neighbors = []
        for i in range(self.n):
            thetasTmp = thetas.copy()
            moveAngle = self.moveAngles[i]
            thetasTmp[i] += moveAngle
            if i+1 < self.n:
                thetasTmp[i+1] -= moveAngle
            jointPos = self.fk(thetasTmp)
            neighbors.append((thetasTmp, jointPos))

            thetasTmp = thetas.copy()
            moveAngle = self.moveAngles[i]
            thetasTmp[i] -= moveAngle
            if i+1 < self.n:
                thetasTmp[i+1] += moveAngle
            jointPos = self.fk(thetasTmp)
            neighbors.append((thetasTmp, jointPos))
        return neighbors

def test():
    r = Robot([1,1,1,1,1,1])
    r.visualize([math.pi/4,0,0,0,0,0])
    for _, jointPos in r.getNeighbors([math.pi/4,0,0,0,0,0]):
        r.visualize(jointPos=jointPos)

if __name__ == '__main__':
    test()

