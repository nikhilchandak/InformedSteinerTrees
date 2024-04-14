from config import *

class DynamicTree:
    def __init__(self, mst):
        self.adj = {}
        self.cost = 0 

        self.cheapestEgde = None 
        self.cheapestCost = float('inf')
        self.visited = 0 
        self.numNodes = len(mst) + 1 # goalCount
        self.mstEdges = []
        
        for i in range(self.numNodes):
            self.adj[i] = {}

        for e in mst :
            u, v = e.u, e.v 
            self.mstEdges.append(edgeNumberMap[(u, v)])

            w = e.getCost()
            self.adj[u][v] = w
            self.adj[v][u] = w 

            self.cost += w 
            self.updateCheapestEdge(u, v, w)
            
        pass 

    def updateEdge(self, e):
        u, v = e.u, e.v 
        now = edgeNumberMap[(u, v)]
        w = e.getCost()

        # Edge already in MST 
        if u in self.adj[v] and v in self.adj[u] : 
            if (w - EPS) > self.adj[u][v]:
                print("Downgrade edge length update:", w, self.adj[u][v]) 

            assert w - EPS <= self.adj[u][v]
            
            self.cost -= (self.adj[u][v] - w)
            self.adj[u][v] = w 
            self.adj[v][u] = w 
            self.updateCheapestEdge(u, v, w)

        else :
            costliestEdge = self.getCostliestEdgeinCycle(u, v)
            if w < costliestEdge[0] :
                
                self.cost -= (costliestEdge[0] - w)

                x, y = costliestEdge[1] 
                    
                self.adj[x].pop(y)
                self.adj[y].pop(x)

                self.adj[u][v] = w 
                self.adj[v][u] = w 

                earlier = edgeNumberMap[(x, y)]
                # print("Debug:", self.mstEdges)
                self.mstEdges.remove(earlier)
                self.mstEdges.append(now)
                # print("Debug:", self.mstEdges)

                self.updateCheapestEdge(u, v, w)

    def getMSTcost(self):
        return self.cost  

    def getCurrentTree(self):
        return self.adj 

    def getMSTedgeIndices(self):
        return self.mstEdges

    def getCostliestEdgeinCycle(self, u, v):
        return self.dfs(u, -1, v, list(self.cheapestEdge))

    def getPathCost(self, u, v):
        return self.dfs5(u, -1, v)

    def dfs5(self, u, p, v):
        if u == v :
            return 0 

        ans = 1e9
        for k in self.adj[u] :
            if k == p :
                continue 
            ans = min(ans, self.adj[u][k] + self.dfs5(k, u, v))

        return ans 

    def updateCheapestEdge(self, u, v, w):
        if w < self.cheapestCost :
            self.cheapestCost = w 
            self.cheapestEdge = [u, v]

    def dfs(self, u, p, v, maxCostEdge):
        original = list(maxCostEdge)
        w = self.adj[maxCostEdge[0]][maxCostEdge[1]]
        if u == v :
            return w, maxCostEdge

        best = (0, [0, 1])
        for k in self.adj[u] : 
            if k == p :
                continue 

            if self.adj[u][k] > w :
                maxCostEdge = [u, k]
            else :
                maxCostEdge = original 

            ret = self.dfs(k, u, v, maxCostEdge)
            if ret[0] > best[0]:
                best = ret 

        return best 

    def getTourCost(self):
        # tour = []
        # self.dfs2(0, -1, tour)

        # tourCost = 0
        # for i in range(1, len(tour)):
        #     tourCost += self.adj[tour[i-1]][tour[i]]

        # return tourCost 
        self.visited = 0 
        return self.dfs3(0, -1, 0) 

    def dfs2(self, u, p, tour):
        tour.append(u)
        for v in self.adj[u]:
            if v == p :
                continue 
            self.dfs2(v, u, tour) 

    def dfs3(self, u, p, cost):
        self.visited += 1 

        for v in self.adj[u]:
            if v == p : 
                continue 
            cost = self.dfs3(v, u, cost + self.adj[u][v])
        
        if self.visited != self.numNodes :
            return cost + self.adj[u][p] 
        else :
            return cost 

