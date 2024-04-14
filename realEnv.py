"""
This file specifies the real vector space (R^n) enviornments for benchmarking.
Inspired from instances used in InformedRRT* paper:
Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic
"""

from ompl import base as ob
import itertools 

import sys
import numpy as np
class RandomWorld:
    obstacleCount = 30 # 50
    maxVolume = 0.75
    finalVolumes = []
    obstacleCenters = []
    maxStrecth = []
    obstacleBounds = [ [] for _ in range(obstacleCount) ]
    maxVolumePerObstacle = maxVolume / obstacleCount 
    maxVolumePerObstacle = 0.05 # 0.03

    def __init__(self, ndim = 4):
        self.dim = ndim 

        self.space = ob.RealVectorStateSpace(ndim)
        self.si = ob.SpaceInformation(self.space)
        self.bounds = ob.RealVectorBounds(ndim)
        self.bounds.setLow(0)
        self.bounds.setHigh(1)
        self.space.setBounds(self.bounds)

        self.called = 0

        if RandomWorld.finalVolumes == [] :

            for i in range(RandomWorld.obstacleCount):
                state = ob.State(self.space)
                state.random()
                for j in range(self.dim):
                    assert (state[j] >= 0 and state[j] <= 1)
                RandomWorld.obstacleCenters.append(state)

                RandomWorld.maxStrecth.append([])
                for j in range(self.dim):
                    RandomWorld.maxStrecth[i].append(min(state[j], 1 - state[j]))

            self.generateObstacles()

    def generateObstacles(self):
        for i in range(RandomWorld.obstacleCount):
            volumeRemaining = RandomWorld.maxVolumePerObstacle
            obstacleVolume = 1 
            squareLength = pow(RandomWorld.maxVolumePerObstacle, 1.0/self.dim)

            for j in range(self.dim):
                maxWidth = 2 * RandomWorld.maxStrecth[i][j] 
                assert maxWidth <= 1 
                
                if j < (self.dim - 1):
                    width = maxWidth * np.random.uniform(0.5, 1) # np.random.random()
                else :
                    width = volumeRemaining / obstacleVolume 
                    width = min(width, maxWidth)

                obstacleVolume *= width 
                RandomWorld.obstacleBounds[i].append(width/2.0)

            # make each axis square aligned (same length) instead 
            obstacleVolume = pow(squareLength, self.dim)
            RandomWorld.obstacleBounds[i] = [squareLength/2.0 for _ in range(self.dim)]

            RandomWorld.finalVolumes.append(obstacleVolume)

            x = self.obstacleCenters[i][0]
            y = self.obstacleCenters[i][1]
            # print(self.obstacleBounds[i])
            deltaX = self.obstacleBounds[i][0]
            deltaY = self.obstacleBounds[i][1] 

            # print(x - deltaX, y - deltaY)
            # # print(x - deltaX, y + deltaY)
            # # print(x + deltaX, y - deltaY)
            # print(x + deltaX, y + deltaY)
            # print()

        # sys.exit()
        # print(sum(RandomWorld.finalVolumes))
        # assert round(sum(RandomWorld.finalVolumes), 5) <= RandomWorld.maxVolume

    def checkCollisionFree(self, state):
        for j in range(RandomWorld.obstacleCount):
            inside = True 
            for i in range(self.dim):
                if state[i] < (RandomWorld.obstacleCenters[j][i] - RandomWorld.obstacleBounds[j][i]) or state[i] > (RandomWorld.obstacleCenters[j][i] + RandomWorld.obstacleBounds[j][i]) :
                    inside = False 
            
            if inside :
                return False 

        self.called += 1 
        return True 

        pass 


class HighDimensionalEnv:
    collisionCheckerDescription = {0: "FreeSpace", 1:"CenterObstacle", 2:"HypercubeEdges", 3:"UniformHyperRectangles", 4:"RandomWorld"}
    collisionCheckerMapping = { val : key for key, val in collisionCheckerDescription.items() }

    def freeSpace(self, state):
        return True 

    def hypercubeEdges(self, state, edgeWidth = 0.3):
        foundMaxDim = False
        for i in range(self.dim - 1, -1, -1):
            if not foundMaxDim : 
                if state[i] > edgeWidth :
                    foundMaxDim = True 
            elif state[i] < (1 - edgeWidth):
                return False 
        return True 
        
    def centerObstacle(self, state, width = 0.45):
        for i in range(0, self.dim):
            if state[i] < (0.5 - width) or state[i] > (0.5 + width) :
                return True 
                # return False 
        self.trueCount += 1 
        # print(state[0], state[1], self.trueCount)
        return False 

    def uniformHyperRectangles(self, state, width=0.1):
        half = width * 0.75 # 0.8 #  0.7 # make it higher for higher dimensions?
        # half = width * pow(0.15, 1.0/self.dim)

        # half = width * pow(0.05, 1.0/self.dim)
        times = int(1.0/width) 
        self.called += 1 

        # orders = itertools.product(list(range(times)), repeat=self.dim)
        orders = [[int(state[i]/width) for i in range(self.dim)]]

        for order in orders:
            flag = True 
            for i in range(self.dim):
                if not (state[i] >= order[i]*width and state[i] <= order[i]*width + half) :
                    flag = False 
            if flag :
                return False 

        return True 

    def __init__(self, ndim=4, stateValidityChecker=2, resolution=1e-4):
        self.trueCount = 0 
        self.called = 0

        self.dim = ndim 
        self.stateValidityChecker = stateValidityChecker 

        if stateValidityChecker not in self.collisionCheckerDescription :
            print("Given environment is not supported! Checker value = ", stateValidityChecker)
            print("Using free space env instead")
            self.stateValidityChecker = 0

        self.space = ob.RealVectorStateSpace(ndim)
        self.spaceCopy = ob.RealVectorStateSpace(ndim)

        self.si = ob.SpaceInformation(self.space)
        self.siCopy = ob.SpaceInformation(self.space)
        
        self.bounds = ob.RealVectorBounds(ndim)
        self.res = resolution 
        
        self.log = False 
        self.randomWorld = RandomWorld(ndim)
        self.funcMap = {0: self.freeSpace, 1: self.centerObstacle, 2: self.hypercubeEdges, 3:self.uniformHyperRectangles, 4:self.randomWorld.checkCollisionFree}

        if self.log :
            print(f"Planning in R^{ndim}")
            print("Using collision checker:", self.collisionCheckerDescription[stateValidityChecker])
            print("Collision Resolution:", self.res)
            print()
        
        self.checker = None 
        self.metaSetup()

    def getTimesCalled(self):
        return self.called 
        
    def isStateValid(self, st):
        try :
            return self.funcMap[self.stateValidityChecker](st)
        except :
            # Assume no obstacle in the whole space 
            return True 

        # if self.stateValidityChecker == 2 :
        #     return hypercubeEdges(st)
    
    def metaSetup(self):
        self.bounds.setLow(0)
        self.bounds.setHigh(1)
        self.space.setBounds(self.bounds)
        self.spaceCopy.setBounds(self.bounds)

        self.si.setStateValidityCheckingResolution(self.res)
        self.checker = ob.StateValidityCheckerFn(self.isStateValid)
        self.checker.isValid = self.isStateValid
        self.si.setStateValidityChecker(self.checker)

    def getStateValidityChecker(self):
        return self.checker

    def getSpace(self):
        return self.space 

    def getSpaceCopy(self):
        return self.spaceCopy

    def getSpaceInformation(self):
        return self.si

    def getSpaceInformationCopy(self):
        return self.setup2.getSpaceInformation()

    def getBounds(self):
        return self.bounds

    def getCollisionResolution(self):
        return self.res
