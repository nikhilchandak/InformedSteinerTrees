from xml.dom import minidom

"""
Specifies a logging system to capture details of the output of the different solvers.

SAMPLE OUTPUT:

<?xml version="1.0" ?>
<Benchmark terminals="10" timeLimit="60" env="APARTMENT" robot="robot_small" runs="50">
  <Sstar edgeOrder="0">
    <output spanningTree="200" tourCost="300" timeTaken="234" pruned="5" optimalEdges="2"/>
  </Star>

  <SFFstar>
  </SFFstar>

  <Baseline>
  </Baseline> 
</Benchmark>
"""

class OutputLogger:
    def __init__(self, numTerminals, timeLimit, env, robot, seed, runs=1):
        self.root = minidom.Document()
        self.benchmark = self.root.createElement('Benchmark')

        self.benchmark.setAttribute("terminals", str(numTerminals)) 
        self.benchmark.setAttribute("timeLimit", str(timeLimit)) 
        self.benchmark.setAttribute("env", env) 
        self.benchmark.setAttribute("robot", robot) 
        self.benchmark.setAttribute("runs", str(runs))
        self.benchmark.setAttribute("seed", str(seed))

        self.mapping = {}
        self.mappingIndex = 10
            
        self.root.appendChild(self.benchmark)
        pass 

    def setupSstar(self, ordering=0, informed=True):
        self.sstar = self.root.createElement("Sstar")
        self.orderingTypeSstar = "UB - LB"
        if ordering == 1 :
            self.orderingTypeSstar = "UB"
        elif ordering == 2 :
            self.orderingTypeSstar = "(UB + LB)/2"

        self.sstar.setAttribute("ordering", self.orderingTypeSstar)
        self.sstar.setAttribute("informed", str(informed))
        self.benchmark.appendChild(self.sstar)
        pass 

    def setupBaseline(self):
        self.baseline = self.root.createElement("Baseline")
        self.benchmark.appendChild(self.baseline)
        pass 

    def setupBetterBaseline(self):
        self.betterBaseline = self.root.createElement("PrunedBaseline")
        self.benchmark.appendChild(self.betterBaseline)
        pass 

    def setupSFFstar(self):
        self.sffstar = self.root.createElement("SFFstar")
        self.benchmark.appendChild(self.sffstar)
        pass 

    def setupOptimal(self):
        self.optimal = self.root.createElement("PotentiallyOptimal")
        self.benchmark.appendChild(self.optimal)
        pass 
    
    def setupLowerBound(self):
        self.lowerBound = self.root.createElement("LowerBound")
        self.benchmark.appendChild(self.lowerBound)
        pass 

    def setupSolver(self, name):
        obj = self.root.createElement(name)
        self.benchmark.appendChild(obj)
    
        self.mapping[self.mappingIndex] = obj
        self.mappingIndex += 1 
        pass

    def saveOutput(self, which, spanningTreeCost, tourCost, timeTaken, pruned=0, remaining=0, costOverTime={}, optimalEdges=0):
        """
        [which]
        Sstar = 0
        SFFstar = 1 
        Baseline = 2 
        BetterBaseline = 3
        PotentiallyOptimal = 5 [Sstar run for 5x time]
        lowerBound = 6 [spanning tree length using edges' lower bound cost]
        """
        solver = None
        output = self.root.createElement("Output")

        if which == 0:
            solver = self.sstar 
        elif which == 1 :
            solver = self.sffstar
        elif which == 2 :
            try :
                solver = self.baseline 
            except :
                solver = self.betterBaseline
        elif which == 3 :
            solver = self.betterBaseline
        elif which == 100 :
            output = self.optimal 
        elif which == 101 :
            output = self.lowerBound
        else :
            solver = self.mapping[which]

        output.setAttribute("spanningTreeLength", str(round(spanningTreeCost, 3)))
        output.setAttribute("tourCost", str(round(tourCost, 3)))
        output.setAttribute("timeTaken", str(round(timeTaken, 3)))

        if pruned > 0:
            output.setAttribute("pruned", str(pruned))
            output.setAttribute("ellipsesRemaining", str(remaining))

        if costOverTime != {} :
            output.setAttribute("costOverTime", str(costOverTime))

        # if which == 0 or which == 2 :
        #     output.setAttribute("optimalEdges", str(optimalEdges))

        if which < 100 :
            solver.appendChild(output)
        pass

    def output(self, filePath=None):
        self.xml_str = self.root.toprettyxml(indent ="\t") 
        # print(self.xml_str)
        
        try :
            f = open(filePath, "w")
            f.write(self.xml_str)
            f.close()
        except :
            pass 

        return self.xml_str  
