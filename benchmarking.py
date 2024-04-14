from modularPRM import *
from sff import * 
from multiTrrt import *
from logger import * 

from timeit import default_timer as timer
from multiprocessing import Pool, cpu_count
from datetime import datetime 
import os 

SEED = 1234
# SEED = 42
# SEED = 150 
ou.RNG.setSeed(SEED)
np.random.seed(SEED)

# from tqdm import tqdm 

DATE_TIME = datetime.now()
PATH_ = "Others"

def generateLogName(numTerminals, envPath, timeLimit, logFolder="Logx", robotPath=""):
    if robotPath != "" :
        robotPath = "_" + robotPath 
    outFile = logFolder + "/" + str(envPath[3:-4]) + "_" + str(numTerminals) + "_" + str(timeLimit) + "s" + str(robotPath) + ".xml"
    print("To Begin:", outFile)
    return outFile

class Benchmarking:

    def __init__(self, numTerminals=5, timeLimit=10, env=RANDOM_POLYGONS_ENV, robot=ROBOT_CYLINDER_SMALL, runs=10, optimalFactor = 3):
        self.numTerminals = numTerminals 
        self.timeLimit = timeLimit 
        self.env = env 
        self.robot = robot 
        self.runs = runs 

        self.dt_now = DATE_TIME 
        self.dt_string = self.dt_now.strftime("%d-%m-%Y %H-%M-%S")
        # self.dt_string = "PrefinalResults" + "/Sstar"
        self.dt_string = PATH_ + "/" + self.dt_string

        cwd = os.getcwd()
        self.logFolder = "Logx"
        self.destinationFolder = os.path.join(cwd, self.logFolder, self.dt_string)
        self.destinationFolderLength = len(self.destinationFolder) + 1
        if not os.path.exists(self.destinationFolder):
            os.makedirs(self.destinationFolder, exist_ok=True)

        self.logFilePath = generateLogName(numTerminals, env, timeLimit, self.destinationFolder)
        self.outputName = self.logFilePath[self.destinationFolderLength:-4]

        self.env2 = env[:-4] + "_blender.obj"
        self.robot2 = robot[:-4] + "_blender.obj"

        # print(self.env2, self.robot2)
        # sys.exit()
        optTime = min(1500, timeLimit)
        # optTime = timeLimit

        self.solverMapping = {}
        self.solverIndex = 10 

        self.logger = OutputLogger(numTerminals, timeLimit, env, robot, SEED, runs)
        self.opt = SamplingBasedStar(numTerminals, optTime, env, robot, 0)
        
        self.sstar = SamplingBasedStar(numTerminals, timeLimit, self.env, self.robot, 0, False)
        self.uninformedSstar = SamplingBasedStar(numTerminals, timeLimit, env, robot, 0, False, False)
        self.incremental_sstar = SamplingBasedStar(numTerminals, timeLimit, self.env, self.robot, 0, True)
        
        self.baseline = Baseline(numTerminals, timeLimit, env, robot)
        self.betterBaseline = PrunedBaseline(numTerminals, timeLimit, env, robot)
        self.baselinePRM = BaselinePRM(numTerminals, timeLimit, env, robot) 

        self.sff = SFFstar(numTerminals, timeLimit, self.env2, self.robot2, self.dt_string)
        self.sff.setParams(self.outputName)
        
        self.mrrt = MultiTRRT(numTerminals, timeLimit, self.env2, self.robot2, self.dt_string)
        self.mrrt.setParams(self.outputName)

        self.orderings = Solver.ORDERINGS 
        self.sstars = []
        for i in range(self.orderings):
            cur = SamplingBasedStar(numTerminals, optTime, env, robot, 0)
            cur.setOrdering(i)
            self.sstars.append(cur)

        self.nodes = self.sstar.getNodes()
        # self.generateValidInstance()
        self.setup()

    def generateValidInstance(self):
        # Find a valid problem instance 
        found = False 
        while not found :
            self.nodes = self.sstar.getNodes()
            self.opt.resetTerminals(self.nodes)
            self.opt.solve()
            if self.opt.getSpanningTreeLength() < 1e8 : 
                found = True 
            else :
                self.sstar = SamplingBasedStar(self.numTerminals, self.timeLimit, self.env, self.robot, 0)

            print("Generating problem for", self.logFilePath, "Found:", found)

    def mapSolver(self, solverr):
        self.solverMapping[self.solverIndex] = solverr
        self.solverIndex += 1

    def setup(self):
        terminals = self.nodes 

        self.uninformedSstar.resetTerminals(terminals)
        self.incremental_sstar.resetTerminals(terminals)
        self.baseline.resetTerminals(terminals)
        self.betterBaseline.resetTerminals(terminals)
        self.baselinePRM.resetTerminals(terminals)
        self.opt.resetTerminals(terminals)

        for i in range(self.orderings):
            self.sstars[i].resetTerminals(terminals)

        self.sff.setSeed(SEED)
        self.sff.resetTerminals(terminals)
        self.sff.setBounds(self.sstar.getBounds())
        self.sff.setCollisionResolution(self.sstar.getCollisionResolution())
        self.sff.setup()
        
        self.mrrt.setSeed(SEED)
        self.mrrt.resetTerminals(terminals)
        self.mrrt.setBounds(self.sstar.getBounds())
        self.mrrt.setCollisionResolution(self.sstar.getCollisionResolution())
        self.mrrt.setup()
        pass 

    def setSstarOrdering(self, val):
        self.sstar.setOrdering(val)

    def setupLogger(self):
        '''
        # self.logger.setupSstar(self.sstar.getOrdering())
        # self.solverMapping[0] = self.sstar 
        '''

        # self.solverMapping[1] = self.sff 
        # self.logger.setupSFFstar()

        # self.logger.setupBaseline()
        # self.logger.setupBetterBaseline()
        self.logger.setupOptimal()
        self.logger.setupLowerBound()

        # self.mapSolver(self.uninformedSstar)
        # self.logger.setupSolver("UninformedSBSstar")

        # self.mapSolver(self.incremental_sstar)
        # self.logger.setupSolver("Ripple")

        # self.mapSolver(self.baselinePRM)
        # self.logger.setupSolver("PRMbaseline")
        
        self.mapSolver(self.sff)
        self.logger.setupSolver("SFFstar")
        
        # self.mapSolver(self.mrrt)
        # self.logger.setupSolver("MultiTRRT")


        # for i, sstar in enumerate(self.sstars):
        #     self.mapSolver(sstar)
        #     name = "S" + str(i)
        #     self.logger.setupSolver(name)
        pass 

    def benchmark(self):
        self.setupLogger()
        spanLowerBound = self.opt.getLowerBoundSpanningLength()
        
        print("Starting benchamrking", self.env, ":", "Terminals:", str(self.numTerminals), "Time:", str(self.timeLimit) + "s", "Lower Bound:", spanLowerBound)
        for i in range(self.runs):
            for j, algo in self.solverMapping.items():
                # print("Doing", j)
                algo.resetTerminals(self.nodes)

                try:
                    algo.solveUsingSstar()
                except :
                    algo.solve()

                pruned = 0
                remaining = self.numTerminals * (self.numTerminals - 1) / 2 

                try :
                    pruned = algo.getPruned()
                    remaining = algo.getRemainingEllipses()
                except :
                    pass 

                self.logger.saveOutput(j, algo.getSpanningTreeLength(), algo.getTourLength(), algo.getTimeTaken(), pruned, remaining, algo.getCostOverTime())
                print(self.logFilePath, " ---- Done with", i, j, ":", algo.getSpanningTreeLength())

        # Save optimal
        # self.logger.saveOutput(100, self.opt.getSpanningTreeLength(), self.opt.getTourLength(), self.opt.getTimeTaken())
        
        # Save lower bound 
        self.logger.saveOutput(101, spanLowerBound, spanLowerBound * 2, 1)
        return True 

    def benchmarkSstar(self):
        self.setupLogger()
        
        print("Starting benchamrking", self.env, ":", "Terminals:", str(self.numTerminals), "Time:", str(self.timeLimit) + "s")
        for i in range(self.runs):
            j = 10 
            algo = self.incremental_sstar 

            algo.resetTerminals(self.nodes)
            sstarCost, sstarOverTime, otherCost, otherOverTime = algo.compareUsingSstar()

            self.logger.saveOutput(j, otherCost, otherCost*2, self.timeLimit, 0, 0, otherOverTime)
            self.logger.saveOutput(0, sstarCost, sstarCost*2, self.timeLimit, 0, 0, sstarOverTime)
            
            print(self.logFilePath, " ---- Done with", i)

        # Save optimal
        # self.logger.saveOutput(100, self.opt.getSpanningTreeLength(), self.opt.getTourLength(), self.opt.getTimeTaken())
        # Save lower bound 
        spanLowerBound = self.opt.getLowerBoundSpanningLength()
        self.logger.saveOutput(101, spanLowerBound, spanLowerBound * 2, 1)
        return True 

    def output(self):
        self.logger.output(self.logFilePath)


def run(numTerminals, timeLimit, envPath, robotPath):
    # Number of trials for each instance (i.e., for 10 nodes, 10 trials, for 30 nodes, 5 trials, for 50 nodes, 5 trials)
    runDict = {10:10, 30:5, 50:5}
    runs = runDict[numTerminals]
    runs = 5

    print("Runs:", runs)
    bench1 = Benchmarking(numTerminals, timeLimit, envPath, robotPath, runs, 1)
    success =  bench1.benchmark()
    # success =  bench1.benchmarkSstar()
    bench1.output()
    print("Completed " + bench1.logFilePath)


if __name__ == "__main__":
    instances = []
    for instance in INSTANCES:
        j = 1
        timesDict = {1:1, 3:3, 5:5}
        for factor in range(1, 6, 2):
            # if factor == 1 :
            #     continue 
            instances.append((instance[0]*factor, instance[1]*timesDict[factor], instance[2], instance[3]))
            j += 1 

    print(tuple(instances))
    # run(10, 60, RANDOM_POLYGONS_ENV, CAR2_ROBOT)
    # run(10, 300, HOME_ENV, HOME_ROBOT)
    # run(30, 600, RANDOM_WORLD, POINT_ROBOT)
    # run(10, 60, HYPER_RECTANGLES_R4, POINT_ROBOT)
    # run(10, 300, HYPER_RECTANGLES, POINT_ROBOT)

    # run(30, 1200, ABSTRACT_ENV, ABSTRACT_ROBOT)
    # run(10, 450, HOME_ENV, HOME_ROBOT)
    # run(30, 1200, HOME_ENV, HOME_ROBOT)
    # run(10, 600, TWISTY_COOL_ENV, TWISTY_COOL_ROBOT)
    # run(30, 1200, TWISTY_COOL_ENV, TWISTY_COOL_ROBOT)
    # run(50, 1800, TWISTY_COOL_ENV, TWISTY_COOL_ROBOT)
    # run(10, 600, DENSE_ENV, ROBOT_CYLINDER_SMALL)
    # run(50, 1800, DENSE_ENV, ROBOT_CYLINDER_SMALL)
    # run(30, 1200, DENSE_ENV, ROBOT_CYLINDER_SMALL)

    started = timer()
    print(f'starting computations on {cpu_count()} cores')
    values = tuple(instances)

    with Pool(3) as pool:
        res = pool.starmap(run, values)
        print(res)

    ended = timer()
    print(f'elapsed time: {ended - started}')