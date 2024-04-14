"""
Main implementation for Informed Steiner Tree (IST*) solver and also contains wrapper for other baselines.
"""

import sys
from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot 
from ompl import util as ou 

# from tqdm import tqdm 
import numpy as np 
import time

SEED = 150
SEED = 123
np.random.seed(SEED)
ou.RNG.setSeed(SEED)

ou.setLogLevel(ou.LogLevel.LOG_ERROR)

from config import * 
from plannerGraph import * 
# from realEnv import *
from environment import *
from problemGenerator import *
from mst import *
from edge import *
from pruning import * 


import cProfile 
import pstats 
from pstats import SortKey

# TSP Tour
import elkai 

class MyStateSampler(ob.StateSampler):
    """
    Implements a custom sampler (to be used by the main algorithm). 
    Its sampleUniform function is called by OMPL if this class is registered to the SpaceInformation class. 

    PS: This is to be only used by the main algorithm (Solver) class. No other solver (like baseline)
    should not require/use this ideally. 

    Some static variables are declared below which are modified by the main algorithm to keep track 
    of whether to grow roadmap and which edge pointer (ellipsoid) to currently sample from. 
    """
    EDGE_PNTR = 0 
    edges = [] 
    GROW_ROADMAP = False 
    sampledIndices = []
    terminals = []
    CALLED = 0 
    si = None
    opt = None 

    def __init__(self, ss):
        super(MyStateSampler, self).__init__(ss)

        self.informedSamplers = []
        self.debug = False 
        self.testing = False 

        # No idea what is the best value to use here. Maybe inf? 
        self.MAX_CALLS = 10000000

        for i in range(len(self.edges)):
            obj = ob.PathLengthDirectInfSampler(self.edges[i].pdef, self.MAX_CALLS)
            self.informedSamplers.append(obj)

    def sampleUniform(self, state):
        """
        While the name of the function is sampleUniform, we override that function with informed sampling! 

        sampledIndices is a list of edge indices such that the distribution of the indices follows the 
        probability distribution as constructed during the current batch in the main algorithm. 

        EDGE_PNTR pops the next edge to be sampled and returns a state lying in its ellipsoid with path length
        upper_bound. 

        The sys.exit() in else should never occur ideally.
        """
        if self.GROW_ROADMAP:
            MyStateSampler.EDGE_PNTR = MyStateSampler.sampledIndices.pop() 

        if self.informedSamplers[self.EDGE_PNTR].sampleUniform(state, ob.Cost(self.edges[self.EDGE_PNTR].upper_bound)) :
            return True 
        else :
            print(self.EDGE_PNTR, "Could not sampled in an informed manner!!")
            print(self.edges[self.EDGE_PNTR].lower_bound, self.edges[self.EDGE_PNTR].upper_bound)

            sys.exit()
            return False 
        

# return an instance of my sampler
def allocMyStateSampler(ss):
     return MyStateSampler(ss)


class Solver:  
    ORDERINGS = 4 
    
    def __init__(self, nodes, edges, si, plannerType=0, incremental=False):
        self.nodes = nodes 
        self.numNodes = len(nodes)

        # self.edges = sorted(edges, key=operator.attrgetter('lower_bound'))
        self.edges = edges 
        self.numEdges = len(edges)

        # BUGGY LINE FOR INFORMED SAMLPING: MyStateSampler.edges = edges 
        # Setup MyStateSampler:
        MyStateSampler.edges = self.edges 
        MyStateSampler.si = si 
        MyStateSampler.opt = ob.PathLengthOptimizationObjective(si)
        MyStateSampler.terminals = [s.getState() for s in self.nodes]
        # print(si.getStateSpace().isMetricSpace())

        self.si = si 
        self.plannerType = plannerType 
        self.data = ob.PlannerData(si)

        terminals = [s.getState() for s in nodes]
        self.terminals = terminals

        self.incremental = incremental
        if incremental :
            self.graph = IncrementalPlannerGraph(terminals, si)
        else :
            self.graph = PlannerGraph(terminals, si)

        if plannerType == 0:
            self.planner = og.PRMstar(si)
        else :
            self.planner = og.LazyPRMstar(si)

        self.planner.setProblemDefinition(edges[0].pdef)
        self.planner.setup()
        
        self.edgeBounds = [1e9 for _ in range(self.numEdges)]
        self.edgeBounds = [self.edges[i].lower_bound for i in range(self.numEdges)]

        self.probabilities = [1.0/(self.numEdges) for _ in range(self.numEdges)]

        self.bestTourCost = 1e9 
        self.edgesFound = []

        self.mst = MinimumSpanningTree(self.numNodes)
        self.dynamic_tree = None 
        self.pruned = 0
        self.pruningStats = []
        self.activeEdges = self.numEdges

        self.startTime = 0
        self.costOverTime = {} 
        self.timeLog = []
        self.lastTimeLog = 0
        self.costLog = []
        self.endTime = 0
        self.timeLimit = 0

        self.ordering = 10
        self.bestCost = float('inf')
        self.timeTaken = 0 

        self.GROW_TIME = 2
        self.EXPAND_TIME = 1 

        self.batchTime = 1 
        self.iterations = 1 

        self.info = True 
        self.log = False 
        pass 

    def setupMSTProbabilityDistribution(self):
        """
        Creates a weighted probability distribution as follows:
        If a MST has not been found yet, give each edge time proportional to its edgeBounds value
        (edgeBound value depends on the ordering used -- can be UB - LB, or just UB or even just LB)
        
        If using edgeBound[edgeIndex] = lower bound then ideally, time should be given only to those 
        edges for which even a single path has not been found yet ie, their upper_bound = INF). This is
        not implemented at the moment.

        If MST exists:
            - Divide the (batch) time between MST and non-MST edges proportional to the number of MST/non-MST edges
            - Give each MST edge time proportional to UB - LB
            - Give each non-MST edge time proportional to UB - the UB of the edge in MST which it can replace  
        """
        # MST exists and self.ordering > 5 indiciates the MST/non-MST time division is used. 
        if self.dynamic_tree != None and self.ordering > 5  :
            mstEdges = self.dynamic_tree.getMSTedgeIndices()
            nonMSTedges = []

            mstCount = 0.0
            mstBounds = 0.0

            for i in mstEdges :
                # Both IF conditions below should be equivalent 
                # if self.edgeBounds[i] > 0 : 
                if not self.edges[i].optimalCostAchieved and not self.edges[i].skip :
                    mstCount += 1 
                    mstBounds += self.edgeBounds[i] 
                    
            nonMSTbounds = 0.0
            nonMSTcount = 0.0

            # Goes through all the non-MST edges 
            for i in range(self.numEdges):
                # Reset probabilities 
                self.probabilities[i] = 0
                if not self.edges[i].optimalCostAchieved and not self.edges[i].skip and i not in mstEdges :
                    nonMSTcount += 1 
                    nonMSTbounds += self.edgeBounds[i] 
                    nonMSTedges.append(i)

            if mstCount == 0 and nonMSTcount == 0:
                # We have achieve the optimal solution!
                pass 
            
            total = mstCount + nonMSTcount 
            
            if mstBounds != 0:
                for i in mstEdges :
                    self.probabilities[i] = (self.edgeBounds[i] / mstBounds) * (mstCount / total)

            if nonMSTbounds != 0 :
                for i in nonMSTedges :
                    self.probabilities[i] = (self.edgeBounds[i] / nonMSTbounds) * (nonMSTcount / total)

            pass 
        else :
            duplicate = np.array(self.edgeBounds)
            self.probabilities = duplicate / np.sum(duplicate)

    def updateProbabilityDistribution(self, SAMPLES_PER_SECOND = 3000):
        """
        SAMPLES_PER_SECOND = Number of nodes sampled, checked for validity to add in PRM per second.
        Calls the probability distrbution setup which is done according to the ordering used. 
        Finally, generates edge indices based on that probability distribution which will be later
        called by the informed Sampler to sample from those edges' ellipsoid. 

        Exception occurs if probability of selecting each edge is set to 0 (or don't add upto 1 for that matter), 
        which should ideally indicate that an optimal solution has been found!
        """
        # SAMPLES = 10000
        SAMPLES = int(self.GROW_TIME * SAMPLES_PER_SECOND)
        self.setupMSTProbabilityDistribution()

        try :
            MyStateSampler.sampledIndices = list(np.random.choice(range(len(self.probabilities)), size=SAMPLES, replace=True, p=self.probabilities))
        except :
            # CAN ONLY HAPPEN WHEN EVERY EDGE IS EITHER OPTIMAL OR PRUNED! 
            # print("All optimal!")
            pass 
        pass 
    
    def checkLemma(self, edgeIndex):
        # Check pruning condition
        i = edgeIndex 
        if self.dynamic_tree != None and not self.edges[i].skip :
            u, v = self.edges[i].u, self.edges[i].v 
            highestCostInCycle = self.dynamic_tree.getCostliestEdgeinCycle(u, v)[0] 

            # NOT >= !!! if self.edges[i].lower_bound >= highestCostInCycle :
            if self.edges[i].lower_bound > highestCostInCycle :
                self.edges[i].skip = True 
                # Whenever we update edgeBounds, we should call updateProbabilityDistribution 
                self.edgeBounds[i] = 0
                self.pruned += 1 
                self.updateProbabilityDistribution()
                return True  
        
        return False  

    def setOrdering(self, val):
        self.ordering = val 

    def updateEdgeBounds(self, idx, regenerate=True):
        try :
            if self.ordering == 0 :
                self.edgeBounds[idx] = self.edges[idx].upper_bound - self.edges[idx].lower_bound 

            elif self.ordering == 1 :
                self.edgeBounds[idx] = 1.0/self.edges[idx].lower_bound # self.edges[idx].upper_bound 

            elif self.ordering == 2 :
                self.edgeBounds[idx] = 1.0/(self.edges[idx].upper_bound - self.edges[idx].lower_bound)

            elif self.ordering == 3 :
                self.edgeBounds[idx] = 1.0/self.edges[idx].upper_bound 

            # elif self.ordering == 5 :
            #     self.edgeBounds[idx] = (self.edges[idx].upper_bound + self.edges[idx].lower_bound) / 2.0 

            else :
                # No MST found yet, give time proportional to its lower bound
                if self.dynamic_tree == None :
                    self.edgeBounds[idx] = self.edges[idx].lower_bound
                else :
                    if idx in self.dynamic_tree.getMSTedgeIndices() :
                        # This edge is in the MST 
                        self.edgeBounds[idx] = self.edges[idx].upper_bound - self.edges[idx].lower_bound 
                    else :
                        # non-MST edge
                        u, v = self.edges[idx].u, self.edges[idx].v 
                        highestCostInCycle = self.dynamic_tree.getCostliestEdgeinCycle(u, v)[0] 
                        self.edgeBounds[idx] = self.edges[idx].upper_bound - highestCostInCycle

        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            if ex == ZeroDivisionError:
                self.edgeBounds[idx] = 0
            else :
                print(message, ex)

        self.edgeBounds[idx] = max(0, self.edgeBounds[idx])
        if regenerate :
            self.updateProbabilityDistribution()

    def growPRM(self):
        if self.plannerType == 0:
            MyStateSampler.GROW_ROADMAP = True 
            self.planner.growRoadmap(self.GROW_TIME)
            if self.dynamic_tree == None :
                self.planner.expandRoadmap(self.EXPAND_TIME)

            MyStateSampler.GROW_ROADMAP = False 
        pass 

    def setBatchTime(self, timeVal):
        self.batchTime = timeVal 

    def setGrowExpandTimes(self, growT, expandT):
        self.GROW_TIME = growT 
        self.EXPAND_TIME = expandT 

    def setNumIterations(self, iterVal):
        self.iterations = iterVal 

    def setupTimings(self, timeLimit):
        """
        Calculates the number of iterations to do and time to give each batch.
        Iterations = sqrt(time)
        Batch Time = time / Iterations 
        """
        sqrt = pow(timeLimit, 0.5)
        self.iterations = int(sqrt)
        self.batchTime = timeLimit // self.iterations  
        self.batchTime /= 2 
        
        self.iterations = timeLimit // self.batchTime


    def checkTimeOver(self, extraTime = 0, subtractTime = 0, OVER = False):
        """
        Checks whether the time limit has been exhausted. 
        If so, do a final run on the edges to see which were/can be pruned. 
        Otherwise, if time remains, log the best solution cost achieved till now. 
        """
        t3 = time.time()
        delta = t3 - self.startTime + extraTime + EPS - subtractTime 

        if OVER or (delta >= self.timeLimit and self.timeLimit != 0) :
            self.timeTaken = t3 - self.startTime 

            self.activeEdges = self.numEdges
            # Do a final count of which all edges can be pruned 
            for i in range(self.numEdges):
                self.checkLemma(i) 
                if self.edges[i].skip or self.edges[i].optimalCostAchieved :
                    self.activeEdges -= 1 

            # print("Remaining ellipsoids:", self.activeEdges)

            for i in range(len(self.timeLog)):
                self.costOverTime[self.timeLog[i]] = self.costLog[i] 
                     
            # if self.timeTaken > self.timeLimit :
            #     print(self.timeTaken, delta, self.timeLimit)
            #     assert 1 == 0 

            return True 
        else :
            if self.bestCost < 1e9 and self.dynamic_tree != None :
                # Easy way (just log):
                # self.costOverTime[round(delta, 4)] = round(self.bestCost, 4)

                # However, easy way can create insane amount of entries in the dictionary with
                # same solution best cost logged at multiple times steps (say t1, t2, t3, ... tN).
                # Instead to save space (as this will be outputted to text files and later read for plotting), 
                # we only log t1 and tN for the same cost solution (ie., the first and last time we had this solution cost)
                curTime = round(delta, 4)
                curCost = round(self.bestCost, 4)
                lastTime = False 
                
                # To achieve this, we make two entries intiially and if the cost is same as earlier, just update the 
                # time of the last entry 
                if self.costLog != [] :
                    if curCost == self.costLog[-1] :
                        self.timeLog[-1] = curTime 
                        lastTime = True 
                
                if not lastTime :
                    self.timeLog.extend([curTime, curTime])
                    self.costLog.extend([curCost, curCost])

                    self.lastTimeLog = curTime
                    # assert curCost - EPS < self.costLog[-1] 

            return False
        pass 

    def checkTimeOverForComparison(self, extraTime = 0, subtractTime = 0, which=0):
        """
        This is a custom function for time logging/checking only to be
        used when superclass calls compareSstar in this class. Assuming ripple 
        (other approach) to be optimal, it logs the best solution costs till 
        the given time for both the apporaches. 

        Checks whether the time limit has been exhausted. 
        If so, do a final run on the edges to see which were/can be pruned. 
        Otherwise, if time remains, log the best solution cost achieved till now. 
        """
        t3 = time.time()
        delta = t3 - self.startTime + extraTime + EPS - subtractTime 

        if delta >= self.timeLimit and self.timeLimit != 0 :
            try :
                if which == 0:
                    if self.otherDone :
                        return True
                else :
                    if self.sstarDone :
                        return True
            except :
                pass 

            self.timeTaken = t3 - self.startTime 

            # Do a final count of which all edges can be pruned 
            for i in range(self.numEdges):
                self.checkLemma(i) 

            if which == 0:
                for i in range(len(self.timeLog)):
                    self.costOverTime[self.timeLog[i]] = self.costLog[i] 
            else :
                for i in range(len(self.timeLog2)):
                    self.costOverTime2[self.timeLog2[i]] = self.costLog2[i] 

            return True 
        else :
            if self.bestCost < 1e9 :
                # Easy way (just log):
                # self.costOverTime[round(delta, 4)] = round(self.bestCost, 4)

                # However, easy way can create insane amount of entries in the dictionary with
                # same solution best cost logged at multiple times steps (say t1, t2, t3, ... tN).
                # Instead to save space (as this will be outputted to text files and later read for plotting), 
                # we only log t1 and tN for the same cost solution (ie., the first and last time we had this solution cost)
                curTime = round(delta, 4)
                curCost = round(self.bestCost, 4)
                lastTime = False 
                
                # To achieve this, we make two entries intiially and if the cost is same as earlier, just update the 
                # time of the last entry 
                if which == 0:
                    if self.costLog != [] :
                        if curCost == self.costLog[-1] :
                            self.timeLog[-1] = curTime 
                            lastTime = True 
                    
                    if not lastTime :
                        self.timeLog.extend([curTime, curTime])
                        self.costLog.extend([curCost, curCost])

                        self.lastTimeLog = curTime
                        # assert curCost - EPS < self.costLog[-1] 

                else :
                    if self.costLog2 != [] :
                        if curCost == self.costLog2[-1] :
                            self.timeLog2[-1] = curTime 
                            lastTime = True 
                    
                    if not lastTime :
                        self.timeLog2.extend([curTime, curTime])
                        self.costLog2.extend([curCost, curCost])

                        self.lastTimeLog2 = curTime

            return False
        pass 

    def checkOptimalAchieved(self):
        # Check if optimal solution found (each edge must be either optimal or pruned) 
        opt = True 
        for e in self.edges :
            if not e.optimalCostAchieved and not e.skip :
                opt = False 
        if opt :
            t2 = time.time()
            self.timeTaken = t2 - self.startTime
        return opt 

    def addTerminalsToPRMgraph(self):
        """
        Explicitly add terminals to the PRM graph by running PRMsolve for a very short amount of time
        """
        self.updateProbabilityDistribution()
        self.initialEdgeTime = 0.001 

        edgePointer = 0 
        while not self.graph.hasAllTerminals :
            i = edgePointer 
            self.planner.setProblemDefinition(self.edges[i].pdef)
            status = self.planner.solve(self.initialEdgeTime)
            self.graph.updatePlannerData(self.planner, True)
            edgePointer = (edgePointer + 1) % self.numEdges 
            
        # ------------------------------------------------------------
        # MyStateSampler.terminals = [s.getState() for s in self.nodes]
        # for i, node in enumerate(self.nodes):
        #     s = ob.PlannerDataVertex(node.getState(), i+1)
        #     self.graph.addVertex(s)
        #     s = s.getState()
        
        # self.planner = og.PRM(self.graph.data)
        # self.planner.setup()

        # self.graph.updatePlannerData(self.planner)
        # self.graph.checkAllTerminals()
        pass 

    def checkMSTfound(self, changePlanner = False):
        # If MST doesn't exist already 
        if self.dynamic_tree == None:
            self.mst.runKruskal(self.edges)
            self.bestCost = min(self.bestCost, self.mst.getMSTcost())

            if self.mst.getMSTcost() < 1e9:
                self.dynamic_tree = DynamicTree(self.mst.getSpanningTree())
                
                # Reupdate the bounds for MST edges (earlier it was LB for them, now UB - LB)
                mstEdges = self.dynamic_tree.getMSTedgeIndices()
                for i in mstEdges:
                    self.updateEdgeBounds(i, False)

                # One a MST has been found, change PRM* to LazyPRM* 
                if changePlanner :
                    self.changePlanner()

    def solvePRMSstar(self, doubling=False, timeLimit=0):
        """
        - Adds termianls to the PRM graph explicitly initially. 
        - Until time exhausted :
            - Update probability distribution of the edges to be sampled next.
            - Grow PRM with informed sampling, as per the probability distribution. 
            - Run S*.
            - For all non-MST edges which are not pruned and haven't reached optimal solution yet (ie, upper bound = lower bound)
                run dijkstra/A* on them. 
            - Update the edge cost and modify MST (add this edge, replace some other cycle edge) if necessary. 
            - Repeat
        """
        if timeLimit != 0 :
            self.iterations = 10000000
            self.timeLimit = timeLimit

        self.initialEdgeTime = 0.001
        self.edgeTime = 0.002
       
        self.setupPRMtimings()
        self.addTerminalsToPRMgraph()
        lastIteration = 0

        t1 = time.time() 
        self.startTime = t1 

        if not self.graph.hasAllTerminals :
            # This should not occur ideally!
            print("Disconnected Graph!!")
            return 

        for iter in range(self.iterations):
            self.updateProbabilityDistribution()
            
            deltaTime = self.GROW_TIME
            if self.dynamic_tree == None :
                deltaTime += self.EXPAND_TIME 

            if self.checkTimeOver(deltaTime):
                break 
                self.GROW_TIME = (self.timeLimit - (time.time() - self.startTime) ) * 0.1
                lastIteration += 1 

            self.growPRM()
            if self.checkTimeOver():
                break 

            self.graph.updatePlannerData(self.planner)
            if self.checkTimeOver():
                break 

            try :
                mstEdges = self.graph.runSstar(False, self)
                lengths = mstEdges.values()
                self.bestCost = min(self.bestCost, sum(lengths))
                self.edgesFound = mstEdges

            except Exception as ex:
                # Ideally, exception should occur only when graph is disconnected and 
                # S* throws nodeQueue empty error correspondingly
                template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                message = template.format(type(ex).__name__, ex.args)
                print(message, ex)
                self.bestCost = 1e9 

            if self.checkTimeOver():
                break 
            
            if self.bestCost < 1e9 :
                for i, edge in enumerate(self.edges):
                    if self.checkOptimalAchieved():
                        return self.checkTimeOver(0, 0, True)

                    if self.edges[i].skip or self.checkLemma(i) or self.edges[i].optimalCostAchieved :
                        continue 

                    e = (edge.u, edge.v)
                    edgeCost = 1e9 

                    if e in mstEdges :
                        edgeCost = mstEdges[e] 
                    else :
                        if self.dynamic_tree != None:
                            edgeCost = self.dynamic_tree.getPathCost(self.edges[i].u, self.edges[i].v)

                    if edgeCost < 1e9 :
                        self.edges[i].updateCost(edgeCost)

                        if self.dynamic_tree != None :
                            self.dynamic_tree.updateEdge(self.edges[i]) 
                            self.bestCost = min(self.bestCost, self.dynamic_tree.getMSTcost())

                        self.updateEdgeBounds(i, False)

                    self.checkMSTfound()
                    
                    if self.checkTimeOver():
                        break 

            if self.checkTimeOver():
                break 

            # print("\nIteration:", iter, "Vertices\Edges in roadmap:", self.graph.node_count(), self.graph.edge_count(), " Spanning Tree length:", self.bestCost, "Pruned:", self.pruned) 
        
        self.runLKH()
        #     print("\nIteration:", iter, "Vertices\Edges in roadmap:", self.graph.node_count(), self.graph.edge_count(), " Spanning Tree length:", self.bestCost, "Pruned:", self.pruned) 
        
        # print("Total nodes:", self.graph.node_count())

        # if self.incremental :
        #     opt = self.graph.optimalMSTlength()
        #     mstcost = self.dynamic_tree.getMSTcost()
        #     dif = abs(opt - mstcost)
        #     if dif > EPS :
        #         print("Optimal:", opt, "Incremental:", mstcost, "Difference:",  dif) 
        #     assert dif <= EPS 
        # print(lastIteration)

        pass 

    
    def compareSstar(self, timeLimit=0):
        """
        Run both naive S* and Ripple.
        """
        if timeLimit != 0 :
            self.iterations = 10000000
            self.timeLimit = timeLimit

        self.initialEdgeTime = 0.001
        self.edgeTime = 0.002
       
        self.setupPRMtimings()
        self.addTerminalsToPRMgraph()

        rippleTime = 0
        sstarTime = 0

        self.otherDone = False 
        self.sstarDone = False 
        
        # set up logging 
        self.timeLog = []
        self.costOverTime = {}
        self.costLog = []
        self.lastTimeLog = 0

        self.costOverTime2 = {} 
        self.timeLog2 = []
        self.lastTimeLog2 = 0
        self.costLog2 = []

        t1 = time.time() 
        self.startTime = t1 

        if not self.graph.hasAllTerminals :
            # This should not occur ideally!
            print("Disconnected Graph!!")
            return 

        while not self.sstarDone or not self.otherDone : 
            self.updateProbabilityDistribution()
            
            deltaTime = self.GROW_TIME
            if self.dynamic_tree == None :
                deltaTime += self.EXPAND_TIME 

            self.otherDone, self.sstarDone = self.checkTimeOverForComparison(deltaTime, sstarTime, 0), self.checkTimeOverForComparison(deltaTime, rippleTime, 1)
            if self.sstarDone and self.otherDone :
                break 

            self.growPRM()
            
            self.otherDone, self.sstarDone = self.checkTimeOverForComparison(0, sstarTime, 0), self.checkTimeOverForComparison(0, rippleTime, 1)
            if self.sstarDone and self.otherDone :
                break 

            self.graph.updatePlannerData(self.planner)

            self.otherDone, self.sstarDone = self.checkTimeOverForComparison(0, sstarTime, 0), self.checkTimeOverForComparison(0, rippleTime, 1)
            if self.sstarDone and self.otherDone :
                break 

            # time ripple 
            timeNow = time.time()
            mstEdges = self.graph.runSstar(False)
            allSum = sum(mstEdges.values())
            rippleTime += time.time() - timeNow

            # time sstar
            timeNow = time.time()

            if self.sstarDone :
                self.bestCost = min(self.bestCost, sum(mstEdges.values()))
            elif allSum < 1e9 :
                try :
                    self.bestCost = min(self.bestCost, self.graph.optimalMSTlength())
                except Exception as ex:
                    template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                    message = template.format(type(ex).__name__, ex.args)
                    # print(message, ex)
                    self.bestCost = 1e9 

            sstarTime += time.time() - timeNow

            # do rest as before 
            self.otherDone, self.sstarDone = self.checkTimeOverForComparison(0, sstarTime, 0), self.checkTimeOverForComparison(0, rippleTime, 1)
            if self.sstarDone and self.otherDone :
                break 

            if self.bestCost < 1e9 :
                for i, edge in enumerate(self.edges):
                    if self.checkOptimalAchieved():
                        return 

                    if self.edges[i].skip or self.checkLemma(i) or self.edges[i].optimalCostAchieved :
                        continue 

                    e = (edge.u, edge.v)
                    edgeCost = 1e9 

                    if e in mstEdges :
                        edgeCost = mstEdges[e] 
                    else :
                        if self.dynamic_tree != None:
                            edgeCost = self.dynamic_tree.getPathCost(self.edges[i].u, self.edges[i].v)

                    if edgeCost < 1e9 :
                        self.edges[i].updateCost(edgeCost)

                        if self.dynamic_tree != None :
                            self.dynamic_tree.updateEdge(self.edges[i]) 
                            self.bestCost = min(self.bestCost, self.dynamic_tree.getMSTcost())

                        self.updateEdgeBounds(i, False)

                    self.checkMSTfound()
                    
                    self.otherDone, self.sstarDone = self.checkTimeOverForComparison(0, sstarTime, 0), self.checkTimeOverForComparison(0, rippleTime, 1)
                    if self.sstarDone and self.otherDone :
                        break 

            self.otherDone, self.sstarDone = self.checkTimeOverForComparison(0, sstarTime, 0), self.checkTimeOverForComparison(0, rippleTime, 1)
            if self.sstarDone and self.otherDone :
                break 
            
            # print("\nVertices\Edges in roadmap:", self.graph.node_count(), self.graph.edge_count(), " Spanning Tree length:", self.bestCost, "Pruned:", self.pruned, "RippleTime:", rippleTime, "SstarTime:", sstarTime, "SstarDone?:", self.sstarDone) 

        return self.costOverTime, self.costOverTime2

    def setupPRMtimings(self):
        if self.plannerType == 0 :
            growT = self.batchTime * 0.7 
            expandT = self.batchTime * 0.3 
            # print("Batchtime:", self.batchTime, "GrowTime:", growT, "Expand Time:", expandT)
            self.setGrowExpandTimes(growT, expandT)         
    
    def solveWithPRM(self, doubling=False, timeLimit=0): 
        """
        BUGGY FUNCTION ALERT!
        Similar to other solve function. Uses PRMsolve to get path lengths instead
        of explicityly running dijkstra. 

        However, it was found that PRMsolve is not giving the same answer as dijkstra.
        Sometimes it gives same answer as dijkstra while other times it gives different (even under the same time limit!)

        I personally doubt that such a bug would exist in OMPL. 
        It might be the case that the time given to find the path is lower than the sufficient time required by Boost.
        This issues remains to be investiagated and fixed!  
        """
        if timeLimit != 0 :
            self.iterations = 10000000
            self.timeLimit = timeLimit

        self.edgeTime = 0.01
        if doubling :
            self.batchTime = min(self.batchTime * self.iterations, self.numEdges * 0.01)
        
        self.setupPRMtimings()

        t1 = time.time() 
        self.startTime = t1 

        for iter in range(self.iterations):
            if self.checkOptimalAchieved():
                return 

            self.updateProbabilityDistribution()
            print(self.bestCost, self.GROW_TIME, self.pruned, "\n")
            if self.plannerType == 0 :
                self.growPRM()
            used = 0 

            for i in range(self.numEdges):
                curTime =  self.batchTime * self.probabilities[i]
                if self.plannerType == 0 :
                    curTime = self.edgeTime 

                # if self.edges[i].optimalCostAchieved or self.edges[i].skip or self.checkLemma(i) or curTime < 0.001:
                # print(i, self.edges[i].lower_bound, self.edges[i].upper_bound, self.edges[i].skip) 

                if self.edges[i].skip or self.edges[i].optimalCostAchieved or self.checkLemma(i)  :
                    continue 
                
                if self.checkTimeOver(curTime):
                    return 

                # print(i, self.edges[i].lower_bound, self.edges[i].upper_bound) 

                used += 1 
                self.planner.setProblemDefinition(self.edges[i].pdef)
                MyStateSampler.EDGE_PNTR = i 

                if self.info:
                    print("Running for ", round(curTime, 3), "seconds")

                self.planner.solve(curTime)
                self.planner.clearQuery()
                
                dijkstra = self.graph.externalDijkstra(self.planner, self.edges[i].u, self.edges[i].v)

                if self.edges[i].pdef.hasExactSolution() :
                    pathLen = self.edges[i].pdef.getSolutionPath().length()

                    assert abs(dijkstra - pathLen) <= EPS 

                    pathLen = min(pathLen, dijkstra)
                    self.edges[i].updateCost(pathLen)
                    # self.edges[i].updateCost(dijkstra)
                    self.updateEdgeBounds(i)
                    
                    # print(i, self.edges[i].lower_bound, self.edges[i].upper_bound) 
                    # print("Expected:", dijkstra)
                    # print()

                    if self.info:
                        print(self.edges[i].u, self.edges[i].v, self.edges[i].upper_bound)

                    if self.dynamic_tree != None :
                        self.dynamic_tree.updateEdge(self.edges[i]) 

                self.checkMSTfound()

            if self.dynamic_tree != None:
                self.pruningStats.append(self.pruned) 

                self.bestCost = self.dynamic_tree.getMSTcost()

                # Uncomment following code for assertions to check Dynamic Tree is working correctly 
                # self.mst.runKruskal(self.edges)
                # delta = self.mst.getMSTcost() - self.dynamic_tree.getMSTcost() 
                # if abs(delta) > EPS :
                #     print(self.mst.getMSTcost(), self.dynamic_tree.getMSTcost())
                # assert abs(delta) <= EPS 

            if doubling :
                self.batchTime *= 2 

            # print("MST Length:", self.bestCost)
            # print("Pruned:", self.pruned, " Skipped:", self.numEdges - used)
            # print("Nodes in PRM:", self.planner.getMilestoneCountString())
            # if self.plannerType == 1 :
            #     print("Roadmap Data:", self.data.numVertices())
            # print()

            if self.checkTimeOver():
                return 
    
    def solve(self, doubling=False, timeLimit=0):
        """
        A cleaner solve function just using Dijkstra on all pairwise valid edges with PRM grow
        """
        assert self.plannerType == 0
        self.timeLimit = timeLimit
        self.edgeTime = 0.01 

        self.setupPRMtimings()
        self.addTerminalsToPRMgraph()

        t1 = time.time() 
        self.startTime = t1 

        while not self.checkTimeOver():
            self.updateProbabilityDistribution()
            # print(self.probabilities)
            self.growPRM()
            
            if self.checkTimeOver():
                break 

            self.graph.updatePlannerData(self.planner)
            
            if self.checkTimeOver():
                break 
            
            for i, edge in enumerate(self.edges):
                if self.checkOptimalAchieved():
                    return 

                if self.edges[i].skip or self.edges[i].optimalCostAchieved or self.checkLemma(i)  :
                    continue 

                edgeCost = self.graph.externalDijkstra(self.planner, self.edges[i].u, self.edges[i].v, True)

                if self.checkTimeOver():
                    return 

                if edgeCost < 1e9 :
                    self.edges[i].updateCost(edgeCost)
                    self.updateEdgeBounds(i, False)

                    if self.dynamic_tree != None :
                        self.dynamic_tree.updateEdge(self.edges[i]) 
                        self.bestCost = min(self.bestCost, self.dynamic_tree.getMSTcost())

                self.checkMSTfound()
                if self.checkTimeOver():
                    break 

            # print("\nVertices in roadmap:", self.graph.node_count(), self.graph.edge_count(), " Spanning Tree length:", self.bestCost, "Pruned:", self.pruned, "AllTerminalsIdentified:", self.graph.hasAllTerminals) 

    def changePlanner(self):
        if self.plannerType == 0 :
            # print("Changing planner!")
            self.plannerType = 1 
            self.planner.getPlannerData(self.data)

            self.planner = og.LazyPRMstar(self.data)

            self.planner.setProblemDefinition(self.edges[0].pdef)
            self.planner.setup()
        
    def getSolutionCost(self):
        return round(self.bestCost, 3)

    def runLKH(self):
        '''
        Use Lin-Kernighan heuristic to get a TSP solution.
        - Currently using 100 iterations of LKH.  
        '''
        ITERATIONS = 100 # LKH iterations 

        # Prepare (symmetric) distance matrix 
        t1 = time.time() 

        distance_matrix = [ [0 for _ in range(self.numNodes) ] for __ in range(self.numNodes) ]

        for i, edge in enumerate(self.edges):
            e = (edge.u, edge.v)
            edgeCost = 1e9 

            if e in self.edgesFound :
                edgeCost = self.edgesFound[e] 
            else :
                if self.dynamic_tree != None:
                    edgeCost = self.dynamic_tree.getPathCost(self.edges[i].u, self.edges[i].v)

            distance_matrix[edge.u][edge.v] = edgeCost 
            distance_matrix[edge.v][edge.u] = edgeCost 
            
        # for e in self.edges :
        #     pair = (e.u, e.v)
        #     if pair in self.edgesFound :
        #         distance_matrix[e.u][e.v] = self.edgesFound[pair] # e.upper_bound 
        #     else :
        #         distance_matrix[e.u][e.v] = self.dynamic_tree.getPathCost(e.u, e.v)
            # distance_matrix[edge.v][edge.u] = distance_matrix[edge.u][edge.v] 
        # print(distance_matrix)

        # Run LKH
        tour = elkai.solve_float_matrix(distance_matrix, ITERATIONS)

        # Obtain tour length 
        tourLength = 0
        for i in range(1, len(tour)):
            tourLength += distance_matrix[tour[i-1]][tour[i]]

        self.bestTourCost = tourLength 

        t2 = time.time() 
        self.timeTaken += t2 - t1

        return tourLength

    def getTourLength(self):
        if self.dynamic_tree == None :
            return 1e9
        else :
            if self.bestTourCost < 1e9 :
                return self.bestTourCost 

            a = round(self.runLKH(), 4)
            b = round(self.dynamic_tree.getTourCost(), 4)
            print("LKH:", a, "Eulerian:", b)
            return a

    def getTimeTaken(self):
        return round(self.timeTaken, 3)

    def getPruned(self):
        return self.pruned 
        
    def getRemainingEllipses(self):
        return self.activeEdges

    def getCostOverTime(self):
        return self.costOverTime
        pass 

    def setInfo(self, val):
        self.info = val 

    def getStatistics(self):
        """
        Planned to write this to get statistics of how (many) edges are pruned over time. 
        Empirically it seems the gains saturate pretty fast.
        No plans to add this function now. Might add it in the future.
        """
        pass 


class SamplingBasedStar:
    """
    This class is the wrapper for our main approach, Informed Steiner Trees. 

    """
    def __init__(self, numTerminals=5, timeLimit=10, ENV_PATH=TRIANG_ENV, ROBOT_PATH = ROBOT_CYLINDER_SMALL, plannerType=1, incremental=False, informedSampling=True):
        self.numTerminals = numTerminals 
        self.timeLimit = timeLimit 
        self.plannerType = plannerType 
        self.incremental = incremental 

        self.mstCost = 1e9
        self.tourLen = 1e9
        self.timeTaken = 0
        self.spanningLowerBound = 0 

        # self.env = Environment(ENV_PATH, ROBOT_PATH)
        # uncomment following line (and comment above line) if you want to use the R^n envorionment 
        # self.env = HighDimensionalEnv(4, 4)
        self.env = metaEnvironment(ENV_PATH, ROBOT_PATH)

        self.space = self.env.getSpace()
        if informedSampling :
            self.space.setStateSamplerAllocator(ob.StateSamplerAllocator(allocMyStateSampler))
        
        self.setup()
        self.prob = None 
        self.createProblemInstance()

    def setup(self):
        self.space2 = self.env.getSpaceCopy()
        self.checker = self.env.getStateValidityChecker()
        self.si = self.env.getSpaceInformation()
        # self.si.setStateValidityCheckingResolution(1.0 / self.space.getMaximumExtent())
        self.si.setStateValidityChecker(self.checker) 

    def createProblemInstance(self):
        def validityFunction(st):
            return self.checker.isValid(st)

        self.prob = ProblemGenerator(self.space2, validityFunction, self.numTerminals)
        self.solver = Solver(self.prob.getNodes(), self.prob.getEdges(), self.si, self.plannerType, self.incremental)
        self.solver.setInfo(False)

    def solve(self, doubling=False):
        """ 
        This function runs the solver which uses dijkstra on each pair of terminals
        with informed sampling.
        """ 

        self.solver.setupTimings(self.timeLimit)
        # solver.setBatchTime(5)
        # solver.setNumIterations(10)

        self.solver.solve(doubling, self.timeLimit)
        self.mstCost = self.solver.getSolutionCost()
        self.tourLen = self.mstCost * 2 # self.solver.getTourLength()
        self.timeTaken = self.solver.getTimeTaken()

        if self.mstCost > 1e9 :
            self.mstCost = 1e9 
            self.tourLen = 1e9 
        # self.solver.solve(0)
        # self.printer() 
        
        
    def compareUsingSstar(self):
        """ 
        This is used to compare ripple (no other apporach is supported at the moment
        through this function) with Sstar.
        """
        self.solver.setupTimings(self.timeLimit)
        otherOverTime, sstarOverTime = self.solver.compareSstar(self.timeLimit)

        try:
            otherCost = list(otherOverTime.values())[-1]
        except :
            otherCost = 1e9 

        try :
            sstarCost = list(sstarOverTime.values())[-1]
        except :
            sstarCost = 1e9 

        return sstarCost, sstarOverTime, otherCost, otherOverTime
        
    def solveUsingSstar(self, doubling=False):
        """ 
        This calls the solver to execute our main approach. 
        Depending whether incremental flag is on/off, Sstar/Ripple 
        approach is used. 
        """
        self.solver.setupTimings(self.timeLimit)
        self.solver.solvePRMSstar(doubling, self.timeLimit)
        self.mstCost = self.solver.getSolutionCost()
        self.tourLen = self.mstCost * 2 #  self.solver.getTourLength()
        self.timeTaken = self.solver.getTimeTaken()

        if self.mstCost > 1e9 :
            self.mstCost = 1e9 
            self.tourLen = 1e9 

    def getSpanningTreeLength(self):
        return self.mstCost

    def getTourLength(self):
        return self.tourLen

    def getTimeTaken(self):
        return self.timeTaken

    def getCostOverTime(self):
        return self.solver.getCostOverTime() 

    def getLowerBoundSpanningLength(self):
        edges = self.solver.edges 
        mst = MinimumSpanningTree(self.numTerminals)
        mst.runKruskalLowerBound(edges)
        self.spanningLowerBound = mst.getMSTcost()
        if self.spanningLowerBound > 1e9 :
            self.spanningLowerBound = 1e9 
        return self.spanningLowerBound

    def printer(self):
        print("Spanning Tree length:", self.mstCost)
        print("Tour Cost:", self.tourLen)
        print("Time Taken:", self.timeTaken)
        print("Pruned:", self.getPruned())
        # print("Cost over time:")
        # print(self.getCostOverTime())
        print()

    def setPlannerType(self, val):
        self.plannerType = val 

    def setNumTerminals(self, val):
        self.numTerminals = val 

    def setTimeLimit(self, val):
        self.timeLimit = val 

    def resetTerminals(self, nodes):
        self.prob.clearNodes()
        self.prob.clearEdges()
        self.prob.nodes = nodes 
        self.prob.generateEdges()
        # self.solver = Solver(self.prob.getNodes(), self.prob.getEdges(), self.si, self.plannerType)
        self.solver = Solver(self.prob.getNodes(), self.prob.getEdges(), self.si, self.plannerType, self.incremental)
        self.solver.setInfo(False)

    def printPairwiseEdges(self):
        if self.prob == None :
            return 
        edges = self.solver.edges 
        k = 0 
        for i in range(len(edges)):
            for j in range(i+1, len(edges)):
                print(edges[k].u, edges[k].v, round(edges[k].getCost(), 3), )
                k += 1 

    def setOrdering(self, val):
        self.solver.setOrdering(val)

    def getOrdering(self):
        return self.solver.ordering
        
    def getBounds(self):
        return self.env.getBounds()

    def getCollisionResolution(self):
        return self.si.getStateValidityCheckingResolution()

    def getNodes(self):
        return self.prob.getNodes()

    def getEdges(self):
        return self.prob.getEdges()

    def getPruned(self):
        return self.solver.getPruned()
        
    def getRemainingEllipses(self):
        return self.solver.getRemainingEllipses()


    def setInfo(self, val):
        self.solver.setInfo(val)

    def __repr__(self):
        pass 

    pass 


class Baseline:
    """
    An abstract baseline class whole solver function implements running a given single-query
    OMPL planner (currently, BIT* but can be changed to AIT/iRRT/ABIT* by giving the appropriate 
    plannerType value during initialization) on each edge. Total time given is divided uniformly
    among all the edges. 
    """
    def __init__(self, numTerminals=5, timeLimit=10, ENV_PATH=TRIANG_ENV, ROBOT_PATH = ROBOT_CYLINDER_SMALL, plannerType=0):
        self.numTerminals = numTerminals 
        self.timeLimit = timeLimit 
        self.plannerType = plannerType 

        self.env_path = ENV_PATH 
        # self.env = Environment(ENV_PATH, ROBOT_PATH)
        # uncomment following line (and comment above line) if you want to use the R^n envorionment 
        # self.env = HighDimensionalEnv(4, 4)
        self.env = metaEnvironment(ENV_PATH, ROBOT_PATH)

        self.space = self.env.getSpace()
        # self.space.setStateSamplerAllocator(ob.StateSamplerAllocator(allocMyStateSampler))
        
        self.metaSetup()
        self.edges = []

        self.setup()
        self.prob = None 
        self.createProblemInstance()

    def metaSetup(self):
        # Reinitialize everything for a new run 
        self.mst = MinimumSpanningTree(self.numTerminals)
        
        self.mstCost = 1e9
        self.tourLen = 1e9
        self.timeTaken = 0
        self.bestCost = 1e9 
        self.pruned = 0

        self.startTime = 0
        self.costOverTime = {} 
        self.endTime = 0
        self.timeLog = []
        self.lastTimeLog = 0
        self.costLog = []

    def setup(self):
        self.space2 = self.env.getSpaceCopy()
        self.checker = self.env.getStateValidityChecker()
        self.si = self.env.getSpaceInformation()
        # self.si.setStateValidityCheckingResolution(1.0 / self.space.getMaximumExtent())
        self.si.setStateValidityChecker(self.checker) 

    def setupEdges(self):
        self.numEdges = len(self.edges)
        for e in self.edges:
            e.setupPlanner(self.plannerType)

    def createProblemInstance(self):
        def validityFunction(st):
            return self.checker.isValid(st)

        self.prob = ProblemGenerator(self.space2, validityFunction, self.numTerminals)
        self.nodes = self.prob.getNodes()
        self.edges = self.prob.getEdges()
        self.numEdges = len(self.edges)

    def setupTimings(self):
        # currently the algorithm is run in batches
        # where number of batches/iterations is sqrt(time) and batchTime = totalTime/batches
        timeLimit = self.timeLimit 
        sqrt = pow(timeLimit, 0.5)
        self.iterations = int(sqrt)
        self.batchTime = timeLimit // self.iterations  
        
        self.iterations = timeLimit // self.batchTime

    def checkTimeOverEarlier(self, extraTime = 0):
        """
        This function is not used anymore.
        Better function for time logging (below) is being used as this might create a lot 
        of redundant entries in costOverTime dictionary
        """
        t3 = time.time()
        delta = t3 - self.startTime + extraTime + EPS
        if delta >= self.timeLimit and self.timeLimit != 0 :
            self.timeTaken = delta 
            return True 
        else :
            if self.bestCost < 1e9 :
                self.costOverTime[round(delta, 4)] = round(self.bestCost, 4)
            return False
        pass 
    
    
    def checkTimeOver(self, extraTime = 0, subtractTime = 0, OVER = False):
        """
        Checks whether the time limit has been exhausted. 
        If so, do a final run on the edges to see which were/can be pruned. 
        Otherwise, if time remains, log the best solution cost achieved till now. 
        """
        t3 = time.time()
        delta = t3 - self.startTime + extraTime + EPS - subtractTime 

        if OVER or (delta >= self.timeLimit and self.timeLimit != 0) :
            self.timeTaken = delta # t3 - self.startTime 

            for i in range(len(self.timeLog)):
                self.costOverTime[self.timeLog[i]] = self.costLog[i] 

            # uncomment this if you want to know how many times was the collision checker 
            # called during the current run of the algorithm. The less, the better :) 

            # try :
            #     print("Collision checker called", self.env.getTimesCalled(), "times")
            # except :
            #     pass

            return True 
        else :
            if self.bestCost < 1e9 :
                curTime = round(delta, 4)
                curCost = round(self.bestCost, 4)
                lastTime = False 
                
                if self.costLog != [] :
                    if curCost == self.costLog[-1] :
                        self.timeLog[-1] = curTime 
                        lastTime = True 
                
                if not lastTime :
                    self.timeLog.extend([curTime, curTime])
                    self.costLog.extend([curCost, curCost])

                    self.lastTimeLog = curTime

            return False

    def solve(self):
        """
        Runs the appropriate solver on each edge with batchTime given
        uniformly among all the edges. Repeat until time over. 
        """
        self.setupEdges()
        self.setupTimings()

        t1 = time.time()
        self.startTime = t1 

        k = 0
        while not self.checkTimeOver():
            curTime = self.batchTime / float(len(self.edges))
            self.edges[k].runSolver(curTime)
            
            self.mst.runKruskal(self.edges)
            self.bestCost = self.mst.getMSTcost()

            k = (k + 1) % self.numEdges
            
        self.mstCost = self.bestCost 
        if self.mstCost < 1e9 :
            self.dynamic_tree = DynamicTree(self.mst.getSpanningTree())
            self.tourLen = self.dynamic_tree.getTourCost()
        else :
            self.mstCost = 1e9 
            self.tourLen = 1e9 
        # self.printer()    

    def printer(self):
        """
        Prints the relevant statistics to judge the performance of the algorithm.
        """
        print("Spanning Tree length:", self.mstCost)
        print("Pruned:", self.pruned)
        print("Tour Cost:", self.tourLen)
        print("Time Taken:", self.timeTaken)
        # print("Cost over time:")
        # print(self.getCostOverTime())
        print()

    def setPlannerType(self, val):
        self.plannerType = val 

    def setNumTerminals(self, val):
        self.numTerminals = val 

    def setTimeLimit(self, val):
        self.timeLimit = val 

    def resetTerminals(self, nodes):
        # Reset edges to prepare for a fresh run of the algorithm 
        # New edges are created from the existing termianls so that 
        # properties from the earlier run of this algorithm don't exist anymore. 

        self.prob.clearNodes()
        self.prob.clearEdges()
        self.prob.nodes = nodes 
        self.prob.generateEdges()

        self.nodes = self.prob.getNodes()
        self.edges = self.prob.getEdges()
        self.numEdges = len(self.edges)
        self.metaSetup()

    def printPairwiseEdges(self):
        if self.prob == None :
            return 
        edges = self.solver.edges 
        k = 0 
        for i in range(len(edges)):
            for j in range(i+1, len(edges)):
                print(edges[k].u, edges[k].v, round(edges[k].getCost(), 3), )
                k += 1 

    def getBounds(self):
        return self.env.getBounds()

    def getCollisionResolution(self):
        return self.si.getStateValidityCheckingResolution()

    def getNodes(self):
        return self.prob.getNodes()

    def getEdges(self):
        return self.prob.getEdges()
        
    def getSpanningTreeLength(self):
        return self.mstCost

    def getTourLength(self):
        return self.tourLen

    def getTimeTaken(self):
        return self.timeTaken

    def getCostOverTime(self):
        return self.costOverTime 

class PrunedBaseline(Baseline):
    """
    Implements algorithm similar to the baseline (above class)
    but prunes edges as per the pruning lemma.  
    """
    def checkLemma(self, edgeIndex):
        # Check pruning condition
        i = edgeIndex 
        if self.dynamic_tree != None :
            u, v = self.edges[i].u, self.edges[i].v 
            highestCostInCycle = self.dynamic_tree.getCostliestEdgeinCycle(u, v)[0] 

            if self.edges[i].lower_bound > highestCostInCycle :
                self.edges[i].skip = True 
                self.pruned += 1 
                return True  
        
        return False  

    def solve(self):
        self.edges = self.prob.getEdges()
        self.setupEdges()
        self.setupTimings()
        
        self.mst.runKruskal(self.edges)

        self.dynamic_tree = None 
        t1 = time.time()
        self.startTime = t1 

        k = 0
        while not self.checkTimeOver():
            if self.edges[k].skip or self.checkLemma(k) :
                k = (k + 1) % len(self.edges)
                continue 

            curTime = self.batchTime / float(len(self.edges))
            self.edges[k].runSolver(curTime)

            if self.dynamic_tree != None :
                self.dynamic_tree.updateEdge(self.edges[k]) 
                self.bestCost = self.dynamic_tree.getMSTcost()
            else :
                self.mst.runKruskal(self.edges)
                if self.mst.getMSTcost() < 1e9 :
                    self.dynamic_tree = DynamicTree(self.mst.getSpanningTree())
                    self.bestCost = self.dynamic_tree.getMSTcost()

            k = (k + 1) % len(self.edges)
            
        if self.dynamic_tree != None :
            self.mstCost = self.dynamic_tree.getMSTcost()
            self.tourLen = self.dynamic_tree.getTourCost()
        else :
            self.mstCost = 1e9 
            self.tourLen = 1e9 

class BaselinePRM(Baseline):
    """ 
    This implements the most naive baseline where a PRM is grown throughout the time
    (samples the whole space uniformly) and in the end S* is run to find MST.

    ISSUE: Time taken by S* is arbitrary so when to stop the growth of PRM such that after running
    S* time taken is same as the timeLimit given during initialization? If PRM is grown till the timeLimit
    and then S* is run then this algorithm may get a superior advantage and even outperform our main algo! 

        I tried to informally regress how much time S* takes depedning upon the size of the PRMgraph 
    (number of nodes). For a few maps, I have manually hardcoded this factor based on my observations. 
    Thus, currently, we calculate how much time S* is expected to take after every batch. If the time 
    passed tilled now (in growing PRM) and the expected time to be taken by S* exceed the timeLimit, then 
    we stop the growth, run S* and get the results. Otherwise, we keep growing PRM. 
    Ofc, this is far from perfect but hopefully such a characterization utilizes all the time and maybe gets 
    slightly advantageous but not much. 
    """

    def __init__(self, numTerminals=5, timeLimit=10, ENV_PATH=TRIANG_ENV, ROBOT_PATH = ROBOT_CYLINDER_SMALL, plannerType=0):
        
        super().__init__(numTerminals, timeLimit, ENV_PATH, ROBOT_PATH, plannerType)

        self.terminals = [s.getState() for s in self.nodes]
        self.graph = PlannerGraph(self.terminals, self.si)
        self.graphCopy = PlannerGraph(self.terminals, self.si)
        self.planner = og.PRMstar(self.si)

    def addTerminalsToPRMgraph(self):
        """
        Explicitly add terminals to the PRM graph by running PRMsolve for a very short amount of time
        """
        self.initialEdgeTime = 0.001 
        if self.env_path == RANDOM_WORLD :
            self.initialEdgeTime = 0.01 

        edgePointer = 0 
        while not self.graph.hasAllTerminals :
            i = edgePointer 
            self.planner.setProblemDefinition(self.edges[i].pdef)
            status = self.planner.solve(self.initialEdgeTime)
            self.graph.updatePlannerData(self.planner, True)
            edgePointer = (edgePointer + 1) % self.numEdges 
            
        pass

    
    def runLKH(self):
        '''
        Use Lin-Kernighan heuristic to get a TSP solution.
        - Currently using 100 iterations of LKH.  
        '''
        ITERATIONS = 100 # LKH iterations 

        # Prepare (symmetric) distance matrix 
        t1 = time.time() 
        distance_matrix = self.graph.APSP()
        t3 = time.time() 
        
        # print("APSP took", round(t3 - t1, 2), "seconds")
        # Run LKH
        tour = elkai.solve_float_matrix(distance_matrix, ITERATIONS)

        # Obtain tour length 
        tourLength = 0
        for i in range(1, len(tour)):
            tourLength += distance_matrix[tour[i-1]][tour[i]]

        self.bestTourCost = tourLength 

        t2 = time.time() 
        self.timeTaken += t2 - t1
        # print("LKH took", round(t2 - t3, 2), "seconds")

        assert tourLength + EPS >= self.mstCost 

        return tourLength


    def solve(self):

        # Hardcoded factors depending upon the map instance 
        mapping = {HOME_ENV: 1500, BARRIERS_EASY_ENV : 1600, ABSTRACT_ENV : 1600, TRIANG_ENV : 1500 }
        factor = 2000.0 # default value 
        TSP_TIME_FACTOR = 1 - (self.numTerminals/150.0)
        TSP_TIME_FACTOR = 0.71 # UH R8 50 temrinals 
        TSP_TIME_FACTOR = 0.7 # CO R8 50 temrinals 

        # if self.env_path in mapping :
        #     factor = float(mapping[self.env_path])
        # self.timeLimit *= TSP_TIME_FACTOR

        self.setupTimings()
        self.batchTime = self.batchTime // 2

        t1 = time.time() 
        self.startTime = t1 

        self.addTerminalsToPRMgraph()
        remainingTime = self.timeLimit - (time.time() - self.startTime)
        requiredTime = 0 
        subtractTime = 0
        # print(remainingTime)
        
        while not self.checkTimeOver(requiredTime, subtractTime) :
            growTime = self.batchTime * 0.9
            self.planner.growRoadmap(growTime)

            requiredTime = self.planner.milestoneCount() / factor 
            requiredTime = 0  # if not running S* after getting out from this loop 
            if self.checkTimeOver(requiredTime, subtractTime):
                break 

            expandTime = self.batchTime * 0.1
            self.planner.expandRoadmap(expandTime)
            # requiredTime = self.planner.milestoneCount() / factor 

            if self.checkTimeOver(requiredTime, subtractTime):
                break 

            # Fetch the PRM graph to run S* next 
            self.graph.updatePlannerData(self.planner)
            
            tNow = time.time() 
            # Run S* for logging purposes 
            try :
                mstEdges = self.graph.runSstar(False, self)
                lengths = mstEdges.values()
                self.bestCost = min(self.bestCost, sum(lengths))
            except Exception as ex:
                self.bestCost = 1e9 

            timeTook = time.time() - tNow 

            # if self.checkTimeOver(timeTook, subtractTime): (BUGGY!?)
            if round(time.time() - self.startTime - subtractTime, 5) + EPS >= self.timeLimit :
                break 
            else :
                subtractTime += timeTook 
                requiredTime = 0
                
            # print("Vertices\Edges in roadmap:", self.graph.node_count(), self.graph.edge_count(), " Spanning Tree length:", self.bestCost, "Time passed:", round(time.time() - self.startTime, 3), "Subtract time:", round(subtractTime, 3)) 
        

        t4 = time.time() 

        # # Fetch the PRM graph to run S* next 
        # self.graph.updatePlannerData(self.planner)

        # # Run S* 
        # try :
        #     mstEdges = self.graph.runSstar(False, self)
        #     lengths = mstEdges.values()
        #     self.bestCost = min(self.bestCost, sum(lengths))

        # except Exception as ex:
        #     # Ideally, exception should occur only when graph is disconnected and 
        #     # S* throws nodeQueue empty error correspondingly
        #     template = "An exception of type {0} occurred. Arguments:\n{1!r}"
        #     message = template.format(type(ex).__name__, ex.args)
        #     print(message, ex)
        #     self.bestCost = 1e9 

        t3 = time.time()
        # self.endTime = t3 
        self.checkTimeOver(0, subtractTime) # log last run if time permits
        # print( round(time.time() - self.startTime - subtractTime, 3) ) 
        self.checkTimeOver(0, subtractTime, True) # exclusively finish time logging

        # print("Nodes in Graph:", self.graph.node_count(), "Time taken by S*", round(self.endTime - t4, 4))

        # self.timeTaken = self.endTime - self.startTime 
        # self.costOverTime = {self.timeTaken : self.bestCost }

        self.mstCost = self.bestCost 
        if self.mstCost < 1e9 :
            self.tourLen = self.mstCost * 2 

            # self.tourLen = self.runLKH()
        else :
            self.mstCost = 1e9 
            self.tourLen = 1e9 
            
        # self.timeLimit /= TSP_TIME_FACTOR 
        

    def resetTerminals(self, nodes):
        super().resetTerminals(nodes)

        self.terminals = [s.getState() for s in self.nodes]
        self.graph = PlannerGraph(self.terminals, self.si)
        self.planner = og.PRMstar(self.si)

        self.bestTourCost = 1e9 

if __name__ == "__main__":
    # 2D
    env = MAZE_PLANNER_ENV
    robo = CAR2_ROBOT

    env = BARRIERS_EASY_ENV
    robo = BARRIERS_EASY_ROBOT

    # env = RANDOM_POLYGONS_ENV
    # robo = CAR2_ROBOT

    # env = BUG_TRAP_PLANNER2D_ENV

    # 3D
    env = ABSTRACT_ENV
    robo = ABSTRACT_ROBOT
    # # env = TRIANG_ENV
    # # robo = ROBOT_CYLINDER_SMALL
    # env = HOME_ENV
    # robo = HOME_ROBOT

    # # R^n
    # env = RANDOM_WORLD
    env = HYPER_RECTANGLES_R4
    robot = POINT_ROBOT

    env = BUILDING_ENV 
    robot = ROBOT_CYLINDER_SMALL 

    nodes = 5
    timing = 300

    # added for python3.7
    pr = cProfile.Profile()
    pr.enable()
    sstar = SamplingBasedStar(nodes, timing, env, robo, 0, True)

    # print(baselineprm.getCostOverTime())

    # # sstar.setInfo(False)
    # # sstar.setOrdering(0)
    # sstar.solveUsingSstar(False)
    sstar.solveUsingSstar()
    sstar.printer()
    # print(sstar.getCostOverTime())
    pr.disable()

    print("MST Lower Bound:", sstar.getLowerBoundSpanningLength())
    
    # baselineprm = BaselinePRM(nodes, timing, env, robo)
    # baselineprm.resetTerminals(sstar.getNodes())
    # baselineprm.solve()
    # baselineprm.printer()
    # print("MST Lower Bound:", sstar.getLowerBoundSpanningLength())
    
    # baseline = Baseline(nodes, timing, env, robo)
    # baseline.resetTerminals(sstar.getNodes())
    # baseline.solve()
    # baseline.printer()


    # import matplotlib.pyplot as plt
    # for algo in [sstar, baselineprm]:
    #     timing = list(algo.getCostOverTime().keys())
    #     costs = list(algo.getCostOverTime().values())

    #     # print(timing, costs)
    #     plt.plot(timing, costs)

    # plt.legend(['IST*', "BaselinePRM"])
    # plt.show()

    # p = pstats.Stats(pr)
    # # p = p.strip_dirs()
    # p.sort_stats(SortKey.CUMULATIVE).print_stats(30)
    # p.sort_stats(SortKey.TIME).print_stats(30)

    ############################### UNCOMMENT BELOW CODE ONLY FOR COMPARING SSTAR WITH ANOTHER (RIPPLE FOR NOW):

    # 2D
    # env = MAZE_PLANNER_ENV
    # robo = CAR2_ROBOT
    # nodes = 10
    # timing = 40
    
    # sstar = SamplingBasedStar(nodes, timing, env, robo, 0, True)

    # a, b, c, d = sstar.compareUsingSstar()

    # print("MST Lower Bound:", sstar.getLowerBoundSpanningLength())
    
    # import matplotlib.pyplot as plt
    # plt.plot(list(d.keys()), list(d.values()))
    # plt.plot(list(b.keys()), list(b.values()))

    # plt.legend(['Ripple', 'S*'])
    # plt.show()
