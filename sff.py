"""
Specifies a wrapper for SFF* (Space Filling Forest*) algorithm to runs its C++ code. 
Relevant codebase: SFF/*
Cloned from: https://github.com/ctu-mrs/space_filling_forest_star
"""

import subprocess 
import pandas as pd 
from xmlConfig import * 
from config import * 
import numpy as np 
import os 

class SFFstar:
    REDIR = "../Instances/"
    MAIN = "SFF/sffstar"
    PARAMS_FILE = "SFF/params.csv"
    DIR = "SFF/"
    columns=['Name','ID','Iterations','Status','Ordering','Edges','SpanningTree','Time']

    def __init__(self, numTerminals=5, timeLimit=10, ENV_PATH=TRIANG_ENV, ROBOT_PATH = ROBOT_CYLINDER_SMALL, dateTime="", plannerType=1):
        self.numTerminals = numTerminals 
        self.timeLimit = timeLimit 
        self.plannerType = plannerType 

        self.env = self.REDIR + ENV_PATH[:-3] + "obj"
        self.robot = self.REDIR + ROBOT_PATH[:-3] + "obj"

        self.collisionResolution = 0.01
        self.uniqueValue = np.random.randint(1, 1000000)
        self.params = "params" + str(self.uniqueValue)
        self.terminalsSet = False 

        self.ndim = 3 
        if '2D' in self.env:
            self.ndim = 2

        self.prob = XMLconfiguration(self.ndim, "sff", "true")
        
        self.DIR += dateTime 
        if not os.path.exists(self.DIR):
            os.makedirs(self.DIR, exist_ok=True)
        if dateTime != "" :
            self.DIR += "/"

        self.mstCost = 1e9
        self.tourLen = 1e9
        self.timeTaken = 0

    def setParams(self, val):
        self.params = val 

    def setup(self):
        self.prob.setRobot(self.robot)
        self.prob.setEnvironment(self.env, self.collisionResolution)
        self.prob.setMaxIterations(int(1e9))
        self.prob.setTimeLimit(self.timeLimit)
        self.params += "_SFF"
        self.prob.setSaveInstructions(self.DIR, self.params)
        pass

    # def setSaveInstructions(self, f=None):
    #     self.prob.setSaveInstructions(f)

    def setSeed(self, seedVal):
        self.prob.setSeed(seedVal)
    
    def resetTerminals(self, nodes):
        if not self.terminalsSet:
            self.prob.setTerminals(nodes)
            self.terminalsSet = True 

    def setBounds(self, bounds):
        self.prob.setBounds(bounds)

    def solve(self):
        temp = "problem_" + self.params + ".xml"
        self.prob.output(self.DIR + temp)

        # res = subprocess.run(["python3", "modularPRM.py"], capture_output=True, text=True)
        # print("----------\n", res.stdout, "\n----------")

        res = subprocess.run(["./" + self.MAIN, "./" + self.DIR + temp], capture_output=True, text=True)
        print(res.stderr)
        self.parseOutput()

    def parseOutput(self):
        toRead = self.DIR + self.params + ".csv"
        df = pd.read_csv(toRead, names=self.columns, header=None)
        self.mstCost = round(df['SpanningTree'].iat[-1], 3)
        self.tourLen = self.mstCost * 2 
        self.timeTaken = round(df['Time'].iat[-1], 3)
        
        status = df['Status'].iat[-1]
        self.status = 1 
        if status == "unsolved" :
            self.status = 0
            self.mstCost = 1e9
            self.tourLen = 1e9 

    def getSpanningTreeLength(self):
        return self.mstCost

    def getTourLength(self):
        return self.tourLen

    def getTimeTaken(self):
        return self.timeTaken

    def getCostOverTime(self):
        return {self.timeTaken : self.mstCost}

    def printer(self):
        print("Spanning Tree length:", self.mstCost)
        print("Tour Cost:", self.tourLen)
        print("Time Taken:", self.timeTaken)

    def setPlannerType(self, val):
        self.plannerType = val 

    def setNumTerminals(self, val):
        self.numTerminals = val 

    def setTimeLimit(self, val):
        self.timeLimit = val 

    def printPairwiseEdges(self):
        pass

    def setCollisionResolution(self, val):
        self.collisionResolution = val

    def __repr__(self):
        pass 

    pass 
