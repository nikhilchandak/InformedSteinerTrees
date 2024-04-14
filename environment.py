"""
Specifies a wrapper on top of OMPL's environment class.
"""

from ompl import app as oa
from ompl import base as ob

class Environment:

    def __init__(self, ENV_PATH='2D/UniqueSolutionMaze_env.dae', ROBOT_PATH = '2D/UniqueSolutionMaze_robot.dae'):
        self.dim = 3
        if '2D' in ENV_PATH:
            self.dim = 2 

        if self.dim == 2 :
            self.setup = oa.SE2RigidBodyPlanning()
            self.setup2 = oa.SE2RigidBodyPlanning()
        else :
            self.setup = oa.SE3RigidBodyPlanning()
            self.setup2 = oa.SE3RigidBodyPlanning()

        self.setup.setEnvironmentMesh(ENV_PATH)
        self.setup2.setEnvironmentMesh(ENV_PATH)
        
        self.setup.setRobotMesh(ROBOT_PATH)
        self.setup2.setRobotMesh(ROBOT_PATH)

        self.log = False 

        if self.log :
            print("Planning in ", ENV_PATH)
            print("Using robot:", ROBOT_PATH)
            print()
        
        self.checker = None 
        self.metaSetup()

    def metaSetup(self):
        start = ob.State(self.setup.getSpaceInformation())
        start.random()
        
        goal = ob.State(self.setup.getSpaceInformation())
        goal.random()
        
        self.setup.setStartAndGoalStates(start, goal)
        self.setup2.setStartAndGoalStates(start, goal)

        self.setup.setup()
        self.setup2.setup()

        self.checker = self.setup.getStateValidityChecker()

    def getStateValidityChecker(self):
        return self.checker  
    
    def isStateValid(self, st):
        return self.checker.isValid(st)

    def getSpace(self):
        return self.setup.getStateSpace()

    def getSpaceCopy(self):
        return self.setup2.getStateSpace()

    def getSpaceInformation(self):
        return self.setup.getSpaceInformation()

    def getSpaceInformationCopy(self):
        return self.setup2.getSpaceInformation()

    def getBounds(self):
        return self.setup.getStateSpace().getBounds()

    def getCollisionResolution(self):
        return self.setup.getSpaceInformation().getStateValidityCheckingResolution()


from realEnv import *

def metaEnvironment(envPath = '2D/UniqueSolutionMaze_env.dae', robotPath = '2D/UniqueSolutionMaze_robot.dae'):
    envName = envPath.split('.')[0] # remove .dae part 
    secondLastChar = envName[-2] # Real environments are name : xxxRn
    thirdLast = envName[-3] # Real env can be xxxRyy

    if secondLastChar == 'R' :
        dim = int(envName[-1]) 
        whichEnv = envName[3:-2]
        checker = HighDimensionalEnv.collisionCheckerMapping[whichEnv]
        obj = HighDimensionalEnv(dim, checker)
        
    elif thirdLast == 'R' :
        dim = int(envName[-2:]) 
        whichEnv = envName[3:-3]
        checker = HighDimensionalEnv.collisionCheckerMapping[whichEnv]
        obj = HighDimensionalEnv(dim, checker)

    else :
        obj = Environment(envPath, robotPath)

    return obj