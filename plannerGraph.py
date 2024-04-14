from steinerpy.algorithms import kruskal
from steinerpy.library.graphs.graph import *
from steinerpy.context import Context

# from steinerpy.library.config import Config as cfg
# cfg.sstar_heuristic_type = "custom"

# from steinerpy.algorithms.common import Common
import steinerpy.config as cfg
cfg.sstar_heuristic_type = "custom"

from steinerpy.algorithms.common import Common

from ompl import base as ob 
from ompl import util as ou
from config import *

import sys 

class PlannerGraph:
    def __init__(self, terminals, si):
        self.si = si 
        self.space = self.si.getStateSpace()
        self.opt = ob.PathLengthOptimizationObjective(si)
        self.data = ob.PlannerData(si)
        self.hasAllTerminals = False 
        self.graph = {}
        self.graph_type = "undirected"

        self.dim = self.si.getStateDimension()
        if self.space.getType() == ob.STATE_SPACE_SE2 :
            self.Rdim = 2 
        elif self.space.getType() == ob.STATE_SPACE_SE3 :
            self.Rdim = 3 
        elif self.space.getType() == ob.STATE_SPACE_REAL_VECTOR :
            self.Rdim = self.dim 

        self.numTerminals = len(terminals)
        self.plannerTerminals = []
        self.originalTemrinals = terminals 
        self.terminalsMap = {}

        self.nodeCount = 0
        self.edgeCount = 0 

        self.edgeMap = ou.vectorUint()
        self.c = ob.Cost()
        pass 

    def neighbors(self, v):
        """
        edgeMap = ou.vectorUint()
        out = data.getEdges(0, edgeMap)
        print(out)
        for x in edgeMap :
            c = ob.Cost()
            cost = data.getEdgeWeight(0, x, c)
            print(x, c.value(), cost)
        """
        v = v[0]
        adj = []

        edgeMap = ou.vectorUint()
        outgoingCount = self.data.getEdges(v, edgeMap)
        for u in edgeMap :
            adj.append((u, -1))
        return adj 
        pass 

    def cost(self, u, v, debug=True):
        u, v = u[0], v[0] 
        dis = self.si.distance(self.data.getVertex(u).getState(), self.data.getVertex(v).getState())

        return dis 

    def node_count(self):
        return self.nodeCount
    
    def edge_count(self):
        return self.edgeCount

    def dijkstra(self, u, v, heuristic=False):
        # Returns the cost of cheapest path from u to v 
        # Naive implementation currently 

        heap = {u:0} 
        dist = {u:0}

        while heap != {} :
            bestKey = list(heap.keys())[0]

            for key in heap.keys():
                if heap[key] <  heap[bestKey] :
                    bestKey = key 

            heap.pop(bestKey)
            leastVal = dist[bestKey]

            if bestKey == v :
                return leastVal 

            neighbors = self.neighbors(bestKey)
            for p in neighbors:
                if p not in dist :
                    dist[p] = 1e9 
                    heap[p] = 1e9 

                if leastVal + self.cost(bestKey, p) < dist[p] :
                    dist[p] = leastVal + self.cost(bestKey, p)
                    heap[p] = dist[p] 
                    # For A*
                    if heuristic :
                        heap[p] += self.cost(p, v) 

        return 1e9 
        pass 

    def externalDijkstra(self, planner, u, v, heuristic = False):
        """
        u, v - terminals
        Finds the mapping of the original terminals to their respective nodes in the PRM graph
        PlannerData is updated from the planner in case new nodes have been added
        If heuristic is enabled, A* search is used instead of Dijkstra 
        """
        self.updatePlannerData(planner)
        if not self.hasAllTerminals :
            return 1e9 

        for key in self.terminalsMap:
            if self.terminalsMap[key] == u :
                u = key 
            
            if self.terminalsMap[key] == v :
                v = key 
        
        val = self.dijkstra(u, v, False)
        astar = self.dijkstra(u, v, True)

        assert abs(val - astar) <= EPS 
        if heuristic :
            return astar 
        return val  

    def dijkstraToAll(self, u):
        # Returns the cost of cheapest path from u to v 
        # Naive implementation currently 

        heap = {u:0} 
        dist = {u:0}
        notFound = list(self.plannerTerminals)

        while heap != {} and len(notFound) > 0 :
            bestKey = list(heap.keys())[0]

            for key in heap.keys():
                if heap[key] <  heap[bestKey] :
                    bestKey = key 

            if bestKey in notFound :
                notFound.remove(bestKey)
                
            heap.pop(bestKey)
            leastVal = dist[bestKey]

            neighbors = self.neighbors(bestKey)
            for p in neighbors:
                if p not in dist :
                    dist[p] = 1e9 
                    heap[p] = 1e9 

                if leastVal + self.cost(bestKey, p) < dist[p] :
                    dist[p] = leastVal + self.cost(bestKey, p)
                    heap[p] = dist[p] 

        distances = {}
        for t in self.plannerTerminals :
            if t in dist :
                distances[(u, t)] = dist[t] 
            else :
                distances[(u, t)] = 1e9 

        return distances 

    def updatePlannerData(self, planner, check=True):
        # self.data.clear()
        planner.getPlannerData(self.data)
        # self.data.decoupleFromPlanner()
        # self.data.computeEdgeWeights()
        self.nodeCount = self.data.numVertices()
        self.edgeCount = self.data.numEdges()
        if check:
            self.checkAllTerminals()
        pass 

    def checkAllTerminals(self):
        """
        Messy right now due to lot of comments added while debugging.
        Checks whether the terminal nodes have been added to the PRM graph. 
        Goes through all the nodes in the PRM graph which is checked against all the terminals. 
        
        Only works for SE(2), SE(3), R^n 
        If using any other configuration space, please add corresponding identification (if) conditions. 
        """
        if self.hasAllTerminals :
            return 

        self.plannerTerminals = []
        found = []

        for i in range(self.nodeCount):
            s = self.data.getVertex(i)
            p = self.data.vertexIndex(s)
            
            if i != p :
                print(s, i, p, " vertex index not matching")
                sys.exit()

            # Safety check (since LazyPRM might throw error while accessing a state with tag 0)
            tag = s.getTag()
            if tag == 0:
                continue 
            s = s.getState()

            for j, terminal in enumerate(self.originalTemrinals):
                matches = True 
                sss = terminal
                
                for idx in range(self.Rdim):
                    try :
                        if self.space.getType() == ob.STATE_SPACE_REAL_VECTOR :
                            if abs(s[idx] - terminal[idx]) > EPS :
                                matches = False
                                break 

                        else :
                            if abs(s[0][idx] - terminal[0][idx]) > EPS :
                                matches = False
                                break 
                    except :
                        print("Accessing RealVector component of the state out of bounds", self.Rdim, idx)
                        break 
                
                if self.space.getType() == ob.STATE_SPACE_SE2 :
                    if abs(s[1].value - terminal[1].value) > EPS :
                        matches = False 
                
                elif self.space.getType() == ob.STATE_SPACE_SE3 :
                    dup = s 
                    s = s.rotation()
                    terminal = terminal.rotation()

                    if not (abs(s.x - terminal.x) < EPS and abs(s.y - terminal.y) < EPS and (s.z - terminal.z) < EPS and (s.w - terminal.w) < EPS ) :
                        matches = False 

                    s = dup
                    
                if matches :
                    if j not in found :
                        self.plannerTerminals.append((i, -1))
                        found.append(j)
                        self.terminalsMap[(i, -1)] = j 

        if len(self.originalTemrinals) == len(self.plannerTerminals):
            self.hasAllTerminals = True 
            
    def addVertex(self, v):
        self.data.addVertex(v)
        self.nodeCount = self.data.numVertices()

    def APSP(self):
        matrix = [ [1e9 for _ in range(self.numTerminals) ] for __ in range(self.numTerminals) ]

        for t in self.plannerTerminals :
            distances = self.dijkstraToAll(t)
            for key, val in distances.items():
                i, j = key 

                u = self.terminalsMap[i]
                v = self.terminalsMap[j]

                matrix[u][v] = val 

                if matrix[v][u] != 1e9 :
                    assert (matrix[u][v] - matrix[v][u]) < EPS 

        return matrix 
        pass 

    def runSstar(self, debug=True, obj=None):
        """
        Returns a dictionary of edges representing the MST (obtained via S*) and their corresponding lengths 
        """
        if not self.hasAllTerminals :
            return 1e9 

        Common.custom_heuristics = self.cost
        context = Context(self, self.plannerTerminals)

        context.run("S*-BS")
        results = context.return_solutions()

        ans = 0
        lengths = results['dist']
        for x in lengths:
            ans += x
        
        edges = results['sol']
        realEdges = []

        # Map nodes (identified as terminals) from PRM graph to their original terminal
        # S* seems to be incorrect at times so if debug is enabled, dijkstra is run to assert correctness  
        for i, edge in enumerate(edges) :
            u = self.terminalsMap[edge[0]]
            v = self.terminalsMap[edge[1]]

            if u > v :
                u, v = v, u 
            
            if debug:
                dijkstra = self.dijkstra(edge[0], edge[1])
                sstar = lengths[i]
                if abs(sstar - dijkstra) > EPS :
                    print("S* giving different answer than Dijkstra!!", u, v, ":", edge, " --- Dijkstra:", dijkstra, "S* :", sstar, "Kruskal :", kruskal_len[i])
                    lengths[i] = dijkstra 

            realEdges.append((u, v))
        
        mst = {}
        for k in range(len(edges)):
            mst[realEdges[k]] = lengths[k] 

        return mst 


from incrementalSstar import * 
class IncrementalPlannerGraph(IncrementalSstarVirtual):
    def __init__(self, terminals, si):
        self.si = si 
        self.space = self.si.getStateSpace()
        self.opt = ob.PathLengthOptimizationObjective(si)
        self.data = ob.PlannerData(si)
        self.hasAllTerminals = False 
        self.graph = {}
        self.graph_type = "undirected"

        self.dim = self.si.getStateDimension()
        if self.space.getType() == ob.STATE_SPACE_SE2 :
            self.Rdim = 2 
        elif self.space.getType() == ob.STATE_SPACE_SE3 :
            self.Rdim = 3 
        elif self.space.getType() == ob.STATE_SPACE_REAL_VECTOR :
            self.Rdim = self.dim 

        self.numTerminals = len(terminals)
        self.plannerTerminals = []
        self.originalTemrinals = terminals 
        self.terminalsMap = {}

        self.bestCost = 1e9 
        self.nodeCount = 0
        self.edgeCount = 0 

        self.edgeMap = ou.vectorUint()
        self.c = ob.Cost()
        
        # For incremental S*
        self.lastNodeCount = 0 
        pass 

    def neighbors(self, v):
        """
        edgeMap = ou.vectorUint()
        out = data.getEdges(0, edgeMap)
        print(out)
        for x in edgeMap :
            c = ob.Cost()
            cost = data.getEdgeWeight(0, x, c)
            print(x, c.value(), cost)
        """
        v = v[0]
        adj = []

        edgeMap = ou.vectorUint()
        outgoingCount = self.data.getEdges(v, edgeMap)
        
        for u in edgeMap :
            if u <= self.lastNodeCount : 
                adj.append((u, -1))
            
        return adj 
        pass 

    def cost(self, u, v, debug=True):
        u, v = u[0], v[0] 
        dis = self.si.distance(self.data.getVertex(u).getState(), self.data.getVertex(v).getState())

        return dis 

    def node_count(self):
        return self.nodeCount
    
    def edge_count(self):
        return self.edgeCount

    def dijkstra(self, u, v, heuristic=False):
        # Returns the cost of cheapest path from u to v 
        # Naive implementation currently 

        heap = {u:0} 
        dist = {u:0}

        while heap != {} :
            bestKey = list(heap.keys())[0]

            for key in heap.keys():
                if heap[key] <  heap[bestKey] :
                    bestKey = key 

            heap.pop(bestKey)
            leastVal = dist[bestKey]

            if bestKey == v :
                return leastVal 

            neighbors = self.neighbors(bestKey)
            for p in neighbors:
                if p not in dist :
                    dist[p] = 1e9 
                    heap[p] = 1e9 

                if leastVal + self.cost(bestKey, p) < dist[p] :
                    dist[p] = leastVal + self.cost(bestKey, p)
                    heap[p] = dist[p] 
                    # For A*
                    if heuristic :
                        heap[p] += self.cost(p, v) 

        return 1e9 
        pass 

    def externalDijkstra(self, planner, u, v, heuristic = False):
        """
        u, v - terminals
        Finds the mapping of the original terminals to their respective nodes in the PRM graph
        PlannerData is updated from the planner in case new nodes have been added
        If heuristic is enabled, A* search is used instead of Dijkstra 
        """
        self.updatePlannerData(planner)
        if not self.hasAllTerminals :
            return 1e9 

        for key in self.terminalsMap:
            if self.terminalsMap[key] == u :
                u = key 
            
            if self.terminalsMap[key] == v :
                v = key 
        
        val = self.dijkstra(u, v, False)
        astar = self.dijkstra(u, v, True)

        assert abs(val - astar) <= EPS 
        if heuristic :
            return astar 
        return val  

    def updatePlannerData(self, planner, check=True):
        planner.getPlannerData(self.data)
        
        self.nodeCount = self.data.numVertices()
        self.edgeCount = self.data.numEdges()
        if check:
            self.checkAllTerminals()
        pass 

    def checkAllTerminals(self):
        """
        Messy right now due to lot of comments added while debugging.
        Checks whether the terminal nodes have been added to the PRM graph. 
        Goes through all the nodes in the PRM graph which is checked against all the terminals. 
        
        Only works for SE(2), SE(3), R^n 
        If using any other configuration space, please add corresponding identification (if) conditions. 
        """
        if self.hasAllTerminals :
            return 

        self.plannerTerminals = []
        found = []

        for i in range(self.nodeCount):
            s = self.data.getVertex(i)
            p = self.data.vertexIndex(s)
            
            if i != p :
                print(s, i, p, " vertex index not matching")
                sys.exit()

            # Safety check (since LazyPRM might throw error while accessing a state with tag 0)
            tag = s.getTag()
            if tag == 0:
                continue 
            s = s.getState()

            for j, terminal in enumerate(self.originalTemrinals):
                matches = True 
                sss = terminal
                
                for idx in range(self.Rdim):
                    try :
                        if self.space.getType() == ob.STATE_SPACE_REAL_VECTOR :
                            if abs(s[idx] - terminal[idx]) > EPS :
                                matches = False
                                break 

                        else :
                            if abs(s[0][idx] - terminal[0][idx]) > EPS :
                                matches = False
                                break 
                    except :
                        print("Accessing RealVector component of the state out of bounds", self.Rdim, idx)
                        break 
                
                if self.space.getType() == ob.STATE_SPACE_SE2 :
                    if abs(s[1].value - terminal[1].value) > EPS :
                        matches = False 
                
                elif self.space.getType() == ob.STATE_SPACE_SE3 :
                    dup = s 
                    s = s.rotation()
                    terminal = terminal.rotation()

                    if not (abs(s.x - terminal.x) < EPS and abs(s.y - terminal.y) < EPS and (s.z - terminal.z) < EPS and (s.w - terminal.w) < EPS ) :
                        matches = False 

                    s = dup
                    
                if matches :
                    if j not in found :
                        self.plannerTerminals.append((i, -1))
                        found.append(j)
                        self.terminalsMap[(i, -1)] = j 

        if len(self.originalTemrinals) == len(self.plannerTerminals):
            self.hasAllTerminals = True 
            super().__init__(self, self.plannerTerminals)
          
    def addVertex(self, v):
        self.data.addVertex(v)
        self.nodeCount = self.data.numVertices()

    def optimalMSTlength(self):
        """
        Returns the optimal MST length using S*-Kruskal / S*-BS 
        """
        if not self.hasAllTerminals :
            return 1e9 

        Common.custom_heuristics = self.cost

        context = Context(self, self.plannerTerminals)
        context.run("S*-BS")
        kruskal_results = context.return_solutions()
        kruskal_len = kruskal_results['dist']

        return sum(kruskal_len)

    def runSstar(self, debug=True, obj=None):
        """
        Returns a dictionary of edges representing the MST (obtained via S*) and their corresponding lengths 
        """
        if not self.hasAllTerminals :
            return {(0, 1) : 1e9}

        # Now run incremental S*
        earlier = self.lastNodeCount
        for i in range(earlier, self.nodeCount):
            self.lastNodeCount = i 

            if (i, -1) not in self.plannerTerminals :
                assert (i, -1) not in self.g.keys() 
                
            super().add((i, -1))

            # check if time over 
            if obj != None :
                if obj.checkTimeOver():
                    return {(0, 1):1e9}

        self.lastNodeCount = self.nodeCount 
        

        if not self.allConnected :
            return {(0, 1) : 1e9}

        grph = {}
        delta = 0 
        realEdges = []

        # Map nodes (identified as terminals) from PRM graph to their original terminal
        for i, data in enumerate(self.shortest_path_values.items()) :
            edge, length = data

            u = self.terminalsMap[edge[0]]
            v = self.terminalsMap[edge[1]]

            if u > v :
                u, v = v, u 
            
            if debug:
                dijkstra = self.dijkstra(edge[0], edge[1])
                sstar = length
                # print(dijkstra, sstar)
                if abs(sstar - dijkstra) > EPS :
                    delta += abs(sstar - dijkstra)
                    print("S* giving different answer than Dijkstra!!", u, v, ":", edge, " --- Dijkstra:", dijkstra, "S* :", sstar)
                    # length = dijkstra 

                # assert abs(sstar - dijkstra) <= EPS

            realEdges.append((u, v))
            grph[realEdges[-1]] = length

        return grph 