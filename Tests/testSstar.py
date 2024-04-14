# from steinerpy import *
from steinerpy.library.graphs.graph import *
from steinerpy.context import Context
import sys

class PQR(IGraph):
    def __init__(self):
        self.readFile()
        self.graph_type = "undirected"      # MUST SPECIFY WHETHER UNDIRECTED OR DIRECTED

    def readFile(self):
        self.graph = {}
        # One can also try testG2.txt or testG1.txt
        # G2prime is a subset of G2 (note that in the file i have specified edges only in one direction assuming
        # they will be interepreted during the file read as bidirectional edges)
        f = open("testG1.txt", "r")

        # Load the graph
        self.nodeCount, self.edgeCount = list(map(int, f.readline().split()))
        print(self.nodeCount, self.edgeCount)
        
        for line in f.readlines():
            u, v, w = list(map(float, line.split()))
            # Make tuples since S* doesn't support single integers 
            u = (int(u), -1)
            v = (int(v), -1) 

            if u not in self.graph :
                self.graph[u] = {}
            if v not in self.graph :
                self.graph[v] = {}

            if u in self.graph[v] :
                assert self.graph[v][u] == w 
            if v in self.graph[u] :
                assert self.graph[u][v] == w 

            self.graph[u][v] = w 
            self.graph[v][u] = w 
            
        # sys.exit()
        pass

    def neighbors(self, v):
        adj = []
        for x in self.graph[v]:
            adj.append(x)
        return adj

    def cost(self, u, v):
        if u in self.graph :
            if v in self.graph[u]:
                return self.graph[u][v]
        # print(u, v)
        return 0

    def node_count(self):
        return len(self.graph.keys())

    def edge_count(self):
        return self.edgeCount
    pass

terminals = [(0, -1), (1, -1)]
graph = PQR()

context = Context(graph, terminals)

context.run('S*-BS')
sstar_results = context.return_solutions()
print(sstar_results)

path = sstar_results['path'][0] 

print()


# HAVE not implemmented path reconstruction for kruskal yet
context.run('Kruskal')
kruskal_results = context.return_solutions()

print("MST VALUES")
print("S*-BS",sum(sstar_results['dist']))
print("Kruskal",sum(kruskal_results['dist']))