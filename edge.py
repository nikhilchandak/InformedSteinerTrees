"""
Specifies a class to maintain edges of the graph induced by the goal nodes in the environment.
The edge class maintains the lower and upper bounds on the edge cost, the optimal cost achieved.
In case a single source and destination planner is being used to solve for every edge (say as a baseline) 
like RRT/RRT*/infomred RRT*/BIT*, then a planner instance can be created for the edge.
"""

from ompl import base as ob 
from ompl import geometric as og
from config import * 

class Edge:
    def __init__(self, terminals, u, v, index, space, isStateValid, lb=0, ub=float('inf')):
        # index of node u & v 
        self.u = u 
        self.v = v

        # lower bound on the optimal edge cost 
        self.lower_bound = lb
        # Upper bound represents the actual realization of the edge cost currently 
        self.upper_bound = ub 

        self.optimalCostAchieved = False 
        self.index = index 

        # Whether this edge can be skipped (ie. pruned) -- can never be part of the MST 
        self.skip = False 

        # construct an instance of space information from this state space
        self.si = ob.SpaceInformation(space)
        # set state validity checking for this space
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

        # BETTER WAY: 
        self.si.setStateValidityCheckingResolution(1.0 / space.getMaximumExtent())
        # self.si.setStateValidityCheckingResolution(1e-3)

        # create a problem instance
        self.pdef = ob.ProblemDefinition(self.si)
        # self.pdef.setOptimizationObjective(getPathLengthObjective(self.si))

        # status of planner 
        self.solved = None 
        self.path = None 

        self.setup(terminals)

    def setup(self, terminals):
        start = terminals[self.u].state 
        goal = terminals[self.v].state 

        self.lower_bound = self.si.distance(start.get(), goal.get())
        # set the start and goal states
        self.pdef.setStartAndGoalStates(start, goal)
        self.si.setup()
        
    def setOptimizationObjective(self, func):
        self.pdef.setOptimizationObjective(func(self.si))

    def setupPlanner(self, plannerType=0):
        # create a planner for the defined space
        if plannerType == 0 :
            self.planner = og.BITstar(self.si)
        elif plannerType == 1 :
            self.planner = og.InformedRRTstar(self.si)
        elif plannerType == 2 :
            self.planner = og.AITstar(self.si)
        elif plannerType == 2 :
            self.planner = og.ABITstar(self.si)

        # set the problem we are trying to solve for the planner
        self.planner.setProblemDefinition(self.pdef)
        # perform setup steps for the planner
        self.planner.setup()

    def runSolver(self, timeLimit=0.1):
        if self.optimalCostAchieved :
            return 

        self.solved = self.planner.solve(timeLimit)
        self.path = self.pdef.getSolutionPath()

        if self.pdef.hasExactSolution():
            # self.path.interpolate()
            self.updateCost(self.path.length())

    # def runPRM(self):
    #     pass 

    def getSolverStatus(self):
        return self.status 

    def isSolved(self):
        return self.optimalCostAchieved 

    def setPath(self, path):
        self.path = path

    def checkOptimality(self):
        if (self.upper_bound - self.lower_bound) <= EPS :
            self.optimalCostAchieved = True 

    def updateCost(self, cost):
        if cost - EPS > self.upper_bound :
            # print("Updating cost with higher length!!! Weird.")
            # print(cost, self.upper_bound)
            # assert cost - EPS <= self.upper_bound 
            return 
            
        self.upper_bound = cost 
        if abs(cost - self.lower_bound) <= EPS :
            self.optimalCostAchieved = True 
            assert (self.lower_bound - cost) <= EPS 
            self.upper_bound = self.lower_bound 

    def getCost(self):
        return self.upper_bound 

    def getPath(self):
        return self.path 
        
    def __repr__(self):
        return f"{self.u} <-> {self.v} : LB = {round(self.lower_bound, 2)}, UB = {round(self.upper_bound, 2)}\n"
