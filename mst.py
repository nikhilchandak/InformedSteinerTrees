"""
Specifies class to find the minimum spanning tree of a graph using Kruskal's algorithm.
This MST is used as a building block to find the Steiner tree of the graph induced by goal nodes. 
"""
import operator 
# from config import * 

class MinimumSpanningTree:

    def __init__(self, numTerminals):
        self.numTerminals = numTerminals
        self.clear() 

    def clear(self):
        self.par = [i for i in range(self.numTerminals)]
        self.size = [1 for i in range(self.numTerminals)]
        self.mst = [] 
        self.highestEdgeWeight = float('inf')
        self.length = 0 
    
    def findPar(self, u):
        if self.par[u] == u : 
            return u 
        else :
            self.par[u] = self.findPar(self.par[u])
            return self.par[u]

    def doUnion(self, u, v):
        parU, parV = self.findPar(u), self.findPar(v)
        if parU == parV :
            return parU 

        if self.size[parU] < self.size[parV] :
            self.par[parU] = parV
            self.size[parV] += self.size[parU]
        else :
            self.par[parV] = parU
            self.size[parU] += self.size[parV]

        return self.par[parU]

    def getSpanningTree(self):
        return self.mst 

    def getHighestEdgeCost(self):
        return self.highestEdgeWeight 

    def getMSTcost(self):
        return self.length 

    def runKruskal(self, edges):
        self.clear() 
        self.edges = sorted(edges, key=operator.attrgetter('upper_bound'))
        for edge in self.edges :
            if self.findPar(edge.u) != self.findPar(edge.v) : 
                self.doUnion(edge.u, edge.v)
                self.length += edge.getCost()
                self.highestEdgeWeight = max(self.highestEdgeWeight, edge.getCost())
                self.mst.append(edge)
                
    def runKruskalLowerBound(self, edges):
        self.clear() 
        self.edges = sorted(edges, key=operator.attrgetter('lower_bound'))
        for edge in self.edges :
            if self.findPar(edge.u) != self.findPar(edge.v) : 
                self.doUnion(edge.u, edge.v)
                self.length += edge.lower_bound
                self.highestEdgeWeight = max(self.highestEdgeWeight, edge.lower_bound)
                self.mst.append(edge)

    def __repr__(self):
        pass 
