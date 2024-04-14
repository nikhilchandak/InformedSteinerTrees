"""
Specifies class to generate problem instances with randomized generation of goal nodes.
"""

import sys
from ompl import base as ob
from terminal import *
from edge import *
from config import *
import operator 

def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

class ProblemGenerator:
    def __init__(self, space, isStateValid, numTerminals, generate=True):
        self.numTerminals = numTerminals 
        self.space = space 
        self.si = ob.SpaceInformation(self.space)
        # set state validity checking for this space
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
        self.si.setStateValidityCheckingResolution(1.0 / space.getMaximumExtent())
        self.si.setup()

        self.MAX_TIMES = 5
        self.checkReachableTime = 60

        self.isStateValid = isStateValid 
        self.nodes = []
        self.edges = []

        self.dim = self.si.getStateDimension()
        if self.space.getType() == ob.STATE_SPACE_SE2 :
            self.Rdim = 2 
        elif self.space.getType() == ob.STATE_SPACE_SE3 :
            self.Rdim = 3 
        elif self.space.getType() == ob.STATE_SPACE_REAL_VECTOR :
            self.Rdim = self.dim 

        self.bounds = self.space.getBounds()

        self.center = ob.State(self.space)
        self.maxStrecth = 0
        for i in range(self.Rdim):
            self.center[i] = (self.bounds.low[i] + self.bounds.high[i])/2.0
            self.maxStrecth += pow(self.bounds.high[i] - self.bounds.low[i], 2)
            
        # Euclidean distance from the two corners of the environment (low, high)
        self.maxStrecth = pow(self.maxStrecth, 0.5)

        self.DISTANCE = self.maxStrecth/self.Rdim
        self.DISTANCE = 1000000
        # print("Raidus:", self.DISTANCE)
        # sys.exit()

        if generate:
            self.generateTerminals()
            self.generateEdges()

    def feasible(self, u, v):
        """
        u: A terminal (state)
        v: Another terminal (state)
        Returns whether there exists a path from u to v  
        """
        # set goal and start states for problem definition 
        self.pdef = ob.ProblemDefinition(self.si)
        self.pdef.setStartAndGoalStates(u, v)
        self.planner = og.RRTConnect(self.si)
        # set the problem we are trying to solve for the planner
        self.planner.setProblemDefinition(self.pdef)
        # perform setup steps for the planner
        self.planner.setup()

        solved = self.planner.solve(self.checkReachableTime)
        status = self.pdef.hasExactSolution()
        if not status :
            return status, 1e9
            
        return status, self.pdef.getSolutionPath().length()

    def generateValidTerminals(self):
        """ 
        Generates a problem instance which is guranteed to be valid (ie., MST exists).
        This is done by ensuring there exists a valid path between the newly added terminal
        and the last terminal (ensured by running RRTconnect for a fixed time). 
        """
        # print("Beginning generation for", self.numTerminals, "terminals\n")
        self.nodes = []
        last = None 
        for i in range(self.numTerminals):
            st = ob.State(self.space)
            st.random()
            if i == 0 :
                last = st 

            timesPathNotFound = 0 

            valid = False
            feasible = False 
            dist = 0
            
            while not valid or not feasible or dist > self.DISTANCE :
                if timesPathNotFound > self.MAX_TIMES :
                    return self.generateTerminals()

                st.random()
                if i == 0 :
                    last = st 
                
                valid = self.isStateValid(st.get())
                dist = self.si.distance(self.center.get(), st.get())
                if not valid or dist > self.DISTANCE:
                    continue 

                feasible, length = self.feasible(last, st)

                # if abs(length - dist) <= EPS :
                #     feasible = False 
                
                if not feasible :
                    timesPathNotFound += 1 
                    
                # print(timesPathNotFound)

            # print(i, last.get(), st.get())
            self.nodes.append(Node(st, i)) 
            last = st

    def generateTerminals(self):
        """
        Just generates collision-free temrinals in the state space. 
        
        The list of terminals depending upon the given environment may or may not
        be in one connected component! 
        """
        # uncomment line below in case the environment has disconnected regions
        # return self.generateValidTerminals()

        self.nodes = []
        last = None 
        for i in range(self.numTerminals):
            st = ob.State(self.space)
            st.random()
            if i == 0 :
                last = st 
            while not self.isStateValid(st.get()) or self.si.distance(self.center.get(), st.get()) > self.DISTANCE :
                st.random()
                if i == 0 :
                    last = st 

            # print(i, last.get(), st.get())
            self.nodes.append(Node(st, i)) 
            last = st

    def generateEdges(self):
        k = 0 
        for i in range(self.numTerminals):
            for j in range(i+1, self.numTerminals):
                self.edges.append(Edge(self.nodes, i, j, k, self.space, self.isStateValid))
                self.edges[-1].setOptimizationObjective(getPathLengthObjective)
                k += 1  

        self.edges = sorted(self.edges, key=operator.attrgetter('lower_bound'))
        for k in range(len(self.edges)):
            i, j = self.edges[k].u, self.edges[k].v 
            edgeNumberMap[(i, j)] = k 
            edgeNumberMap[(j, i)] = k 
            
    def getNodes(self):
        return self.nodes 

    def getEdges(self):
        return self.edges 

    def clearEdges(self):
        self.edges = []

    def clearNodes(self):
        self.nodes = []

    def changeNumTerminals(self, newCount):
        self.numTerminals = newCount 

    def __repr__(self):
        pass 
