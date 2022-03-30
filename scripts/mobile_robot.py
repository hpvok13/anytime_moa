import numpy as np
from math import sin, cos

##################
# Parameters
##################

NUM_ANGULAR_VELOCITIES = 6
TRANSLATION_SPEED = 1

class ShipBot:
    def __init__(self, length, width, wheelRadius):
        self.length = length
        self.width = width
        self.wheelRadius = wheelRadius

    #     translations = np.array([[1, 0],
    #                         [1, 1],
    #                         [0, 1],
    #                         [-1, 1],
    #                         [-1, 0],
    #                         [-1, -1],
    #                         [0, -1],
    #                         [1, -1]])

    #     translations = TRANSLATION_SPEED*translations/np.atleast_2d(np.linalg.norm(translations, axis=1)).T

    #     angularVelocities = np.atleast_2d(np.linspace(-1, 1, NUM_ANGULAR_VELOCITIES)).T

    #     actionsList = []

    #     for i in range(translations.shape[0]):
    #         translationRepeat = np.atleast_2d(translations[i,:]).repeat(NUM_ANGULAR_VELOCITIES, axis=0)
    #         actionsTmp = np.concatenate(translationRepeat, angularVelocities)
    #         actionsList.append(actionsTmp)

    #     self.unitActions = np.concatenate(actionsList, axis=0)
    
    # def transformMatrix(self, pose):
    #     return np.array([[cos(pose[2]), -sin(pose[2]), pose[0]],
    #                      [sin(pose[2]), cos(pose[2]), pose[1]],
    #                      [0, 0, 1]])

    # def getActions(self, transVel, maxAngVel):
    #     return self.unitActions*np.array([transVel, transVel, maxAngVel])

    # def getPoseSucessors(self, poseCur, actionDuration, transVel, maxAngVel):
    #     actions = self.getActions(transVel, maxAngVel)
    #     robotPoseSucessors = actions*actionDuration
    #     T = self.transformMatrix(poseCur)
    #     poseSuccessors = []
    #     for robotPoseSuccessor in list(robotPoseSucessors):
    #         poseSuccessor = T@np.array([robotPoseSuccessor[0], robotPoseSuccessor[1], 1])
    #         poseSuccessor[2] = poseCur[2] + robotPoseSuccessor[2]
    #         poseSuccessors.append(poseSuccessor)
        
    #     return poseSuccessors, list(actions)


    

def test():
    r = ShipBot(20, 10, 5)


if __name__ == '__main__':
    test()