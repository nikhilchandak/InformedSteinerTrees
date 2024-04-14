"""

This module implements the ripple algorithm which basically runs
a dijkstra from the newly added sample and tries to update 
edge lengths of the MST in metric completion if a path connecting
two different terminals is found during the dijkstra. 

"""
from multiprocessing import Value
from typing import List, Iterable, Tuple
from collections import OrderedDict
from heapq import heappush, heappop
import numpy as np

class IncrementalSstarVirtual():
    """
    IncrementalSstar initially took inspiration from S* but
    has now diverged to an independent framework implementing 
    the ripple algorithm. 
    """
    def __init__(self, graph, terminals):

        # keep track of shortest path values between terminals (roots)
        self.shortest_path_values = {}

        # map terminals (order matters)
        self.terminals = terminals           
        self.terminal_index = {}
        for ndx, t in enumerate(terminals):
            self.terminal_index[t] = ndx

        self.g = OrderedDict({ x: 0 for x in self.terminals})
        self.children = {x : set() for x in self.terminals}
        self.parent = {x: x for x in self.terminals}
        self.root = {x: x for x in self.terminal_index} 

        self.par = {x: x for x in self.terminals}
        self.size = {x: 1 for x in self.terminals}
        self.connected_componentes = len(self.terminals)
        self.allConnected = False 

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

    def add(self, pt: tuple):
        """
        Add a randomly sampled node to the data structures of S*
        """
        # ripple instead! 
        self.fastRipple(pt)
        return 

    def fastRipple(self, pt: tuple):
        """
        Start dijkstra from the newly added node. 
        When expanding neighbors, if the neigbor belongs to a different root, see if the path from current to the
        other root can be updated. 
        """

        # Check if the node to be added already exists 
        if pt not in self.g :
            # initialize values
            min_g_n = None
            min_root = None
            self.g[pt] = np.inf

            # find node with minimum gcost+cost(pt, n)
            for n in self.neighbors(pt):
                # neighbors must be part of a tree
                if n in self.root:
                    curr_cost = self.g[n]+self.cost(pt, n)
                    if (min_g_n is None or curr_cost < min_g_n) and self.g[n] < np.inf:
                        # the min g value
                        min_g_n = curr_cost     
                        # the root node corresonding to the new parent node
                        min_root = self.root[n]
                        # set gcost of pt
                        self.g[pt] = min_g_n

            if min_root == None :
                return 

            self.root[pt] = min_root

        else :
            min_g_n = self.g[pt] 
            min_root = self.root[pt]


        dist = {pt: min_g_n}
        heap = []
        heappush(heap, (min_g_n, pt))

        while heap:
            pathCost, best = heappop(heap)

            self.root[best] = min_root
            self.g[best] = dist[best]

            if pathCost > dist[best] :
                continue 
            
            for n in self.neighbors(best):
                if n not in dist :
                    dist[n] = self.g[n]

                gap = self.cost(best, n)

                if dist[best] + gap < dist[n]:
                    dist[n] = dist[best] + gap
                    heappush(heap, (dist[n], n))

                else :
                    if n in self.root and self.root[n] != min_root:
                        self.try_update_paths(min_root, self.root[n], self.g[n] + self.g[best] + gap)
        pass 


    def neighbors(self, pt)->List[tuple]:
        """Return neighbors within a ball centered at pt.
        
        Must return a list of tuple, each tuple is a point!
        
        """
        pass
           
    def cost(self, pt1: tuple, pt2: tuple)->float:
        """Return the cost of an edge.

        Must return a float!

        """
        pass

    def try_update_paths(self, u: tuple, v: tuple, candidate_cost: float):
        """Update shortest path costs if able!
        
        Params:
            u: root1
            v: root2
        Returns:
            1: if updated a path
            0: did not update any path
        """
        # since undirected, simply flip primary and secondary roots 
        # to maintain order 
        if u > v:
            u, v = v, u

        # able to initialize or check if cost is cheaper
        doesntExist = (u,v) not in self.shortest_path_values
        
        if doesntExist or candidate_cost < self.shortest_path_values[(u,v)]:

            if doesntExist and not self.allConnected :
                if self.findPar(u) != self.findPar(v) : 
                    self.doUnion(u, v)
                    self.connected_componentes -= 1
                     
                    if self.connected_componentes == 1 :
                        self.allConnected = True 

            # print((u, v), ":", candidate_cost, "::", self.dijkstra(u, v))
            self.shortest_path_values[(u,v)] = candidate_cost

            return 1
        else:
            # when no change
            return 0