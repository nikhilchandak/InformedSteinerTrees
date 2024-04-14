import xml.etree.ElementTree as ET
from xml.dom import minidom

from matplotlib import pyplot as plt
from benchmarking import generateLogName
import numpy as np
import scipy.stats as st
from config import * 
import sys 
import os 

import pandas as pd
import seaborn as sns


def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), st.sem(a)
    h = se * st.t.ppf((1 + confidence) / 2., n-1)
    return m, h

class OutputPlotter:

    def __init__(self, outputFilePath):
        # self.root = ET.parse(outputFilePath).getroot()
        self.doc = minidom.parse(outputFilePath)

        self.sstar = []
        self.sff = []
        self.baseline = []
        self.baselinePruned = []
        self.solved = []
        self.usbsstar = []

        self.numAlgos = 4 
        self.names = ["Sstar", "SFFstar", "Baseline", "PrunedBaseline"]

        self.numAlgos = 4
        self.names = ["SFFstar", "Ripple", "PRMbaseline", "MultiTRRT"]

        # if "Sstar" in outputFilePath :
        #     self.names = ["Ripple", "Sstar"]

        # self.numAlgos = 4
        # self.names = ["S0", "S1", "S2", "S3"]
        
        self.algos = [[] for _ in range(self.numAlgos)]
        self.tspCosts = [[] for _ in range(self.numAlgos)]
        self.costOverTimes = [{} for _ in range(self.numAlgos)]
        self.costsOverTime = [{} for _ in range(self.numAlgos)]

        self.numTerminals = 0
        self.runs = 0
        
        self.outputName = "" 
        self.lowerBound = 0 
        self.title = "" 

        self.unsolved = False 

        self.colors = ['Red','Orange', 'Blue', 'Purple'] # lavender # lightpink
        self.colors = ['lavender', 'aquamarine', 'lightgrey', 'lightskyblue']
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c'] # default color cycle of matplotlib
        pass 

    
    def extrapolateDictionary(self, diction):
        xaxis = np.arange(0, self.timeLimit + 0.1, 0.1)
        newMap = {}
        bestCost = 1e9 

        for key in diction:
            val = diction[key] 
            newKey = round(key, 1)
            
            # ideally this should be satisfied 
            if newKey > self.timeLimit :
                continue 

            if newKey in newMap :
                newMap[newKey] = min(newMap[newKey], val)
            else :
                newMap[newKey] = val 

        for i in xaxis :
            i = round(i, 1)
            valHere = 1e9 
            if i in newMap :
                valHere = newMap[i]

            bestCost = min(bestCost, valHere)
            if bestCost < 1e9 :
                newMap[i] = bestCost 
        
        return newMap
        pass 
    
    def parseOutputFile(self):
        # algos = [self.sstar, self.sff, self.baseline, self.baselinePruned]
        # algos = [self.sstar, self.usbsstar]
        
        benchmark = self.doc.getElementsByTagName("Benchmark")[0]

        self.env = benchmark.attributes['env'].value
        robot = benchmark.attributes['robot'].value
        terminals = benchmark.attributes['terminals'].value 
        self.timeLimit = int(benchmark.attributes['timeLimit'].value)

        self.outputName = generateLogName(terminals, self.env, self.timeLimit)[5:-4]

        self.runs = int(benchmark.attributes['runs'].value)
        self.numTerminals = int(terminals)


        self.lowerBound = round(float(self.doc.getElementsByTagName("LowerBound")[0].attributes['spanningTreeLength'].value), 2)
        self.averagePruned = 0 
        self.ellipsesRemaining = 0

        for i in range(self.numAlgos):
            # print(self.names[i])
            try :
                algo = self.doc.getElementsByTagName(self.names[i])[0] 
            except :
                
                self.solved.append(self.runs)
                self.unsolved = True 
                continue 

            outputs = algo.getElementsByTagName("Output")

            unsolved = 0 
            for output in outputs:
                val = round(float(output.attributes['spanningTreeLength'].value), 4)
                pathCost = round(float(output.attributes['tourCost'].value), 5)

                # if val > self.lowerBound * 10 and val < 1e9 :
                #     continue 

                if val < 1e9 :
                    try :
                        costOverTime = eval(output.attributes['costOverTime'].value)

                        if len(costOverTime.keys()) == 1 :
                            val = list(costOverTime.values())[0] 
                            costOverTime = {self.timeLimit : val}

                        extrapolatedCostOverTime = self.extrapolateDictionary(costOverTime)

                        self.costOverTimes[i][val] = costOverTime 
                        self.algos[i].append(val)
                        self.tspCosts[i].append(pathCost)

                        if val not in self.costsOverTime[i] :
                            self.costsOverTime[i][val] = []

                        self.costsOverTime[i][val].append(extrapolatedCostOverTime)
                    except :
                        pass
                else :
                    unsolved += 1 

            
            if unsolved > 0:
                self.unsolved = True 

            self.solved.append(unsolved)



        edgeCount = ( self.numTerminals * ( (self.numTerminals - 1) * 0.5) ) - (self.numTerminals - 1)
        self.title = str(self.env[3:-4]) + ":  Terminals = " + str(self.numTerminals) + "  Time = " + str(self.timeLimit) + "s" + "  Runs = " + str(self.runs)
        
        pass 


    def plotDistributionOverTime(self):
        self.fig6, self.ax6 = plt.subplots()
        indices = []
        legendAxes = []


        for i in range(self.numAlgos):
            
            timings = np.arange(0, self.timeLimit + 0.1, 0.1)
            timings = [round(t, 1) for t in timings]

            allCostsOverTime = {t:[] for t in timings}

            for val in self.costsOverTime[i]:
                for distribution in self.costsOverTime[i][val] :
                    for key, costVal in distribution.items() :
                        allCostsOverTime[key].append(costVal)

            medians = []
            upperConfidence = []
            lowerConfidence = []
            xaxisTimes = []

            for timing in timings:
                if timing not in allCostsOverTime :
                    continue 

                allCostsOverTime[timing] = sorted(allCostsOverTime[timing])
                here = allCostsOverTime[timing]

                if len(here) < self.runs :
                    continue 
                # if len(here) < 25 :
                #     continue 

                # medians.append(np.median(here))
                medians.append(np.mean(here))
                xaxisTimes.append(timing)

                n = len(timings)
                # lower CI index 
                # j = math.floor(0.5 * n - 0.98 * math.sqrt(n) )
                # # upper CI index 
                # k = ??
                
                data = here 

                # m, low, high = mean_confidence_interval(data)
                if len(data) < 30 :
                    low, high = st.t.interval(alpha=0.99, df=len(data)-1, loc=np.mean(data), scale=st.sem(data)) 
                else :
                    low, high = st.norm.interval(alpha=0.99, loc=np.mean(data), scale=st.sem(data))

                upperConfidence.append(high)
                lowerConfidence.append(low)

            # print(timing, costs)
            if len(medians) == 1 :
                print("F")
                # plt.axhline(y=costs[0], linestyle='--', color='red')
                # plt.plot(timing, costs, marker='o', markersize=4)

                # plt.plot(xaxisTimes, medians)
                # plt.fill_between(xaxisTimes, lowerConfidence, upperConfidence, alpha=0.1)
                
                plt.plot(xaxisTimes, medians, marker='o', markersize=4, color='orange')
                plt.axhline(y=lowerConfidence[0], linestyle='--', color='orange', alpha=0.5)
                plt.axhline(y=upperConfidence[0], linestyle='--', color='orange', alpha=0.5)

                delta = [upperConfidence[0] - medians[0]]

                plt.errorbar(xaxisTimes, medians, yerr=delta, fmt='o', capthick=2, color='orange', elinewidth=1)
            else :
                p1 = plt.plot(xaxisTimes, medians, color=self.colors[i])
                plt.fill_between(xaxisTimes, lowerConfidence, upperConfidence, alpha=0.1, color=self.colors[i])

                p2 = self.ax6.fill(np.NaN, np.NaN, color=self.colors[i], alpha=0.1)
                legendAxes.append((p2[0], p1[0]))

        # plt.title(self.title)
        plt.ylabel("Solution Cost")
        plt.xlabel("Time (in secs)")

        # plt.legend(self.names, prop={'size': 12})
        names = list(self.names) 
        if names[1] == 'PRMbaseline' :
            names = ['IST*', 'Baseline']

        plt.legend(legendAxes, names, prop={'size': 20})

        plt.tight_layout()
        plt.savefig("Plots/Comparisons/" + self.outputName + "_costDistributionOverTime.png", dpi=400)
        plt.close()

        pass 

    def plotOverTime(self):
        self.fig5, self.ax5 = plt.subplots()
        indices = []

        for i in range(self.numAlgos):
            # print(self.costOverTimes)
            mstLengths = list(self.costOverTimes[i].keys())

            # if len(indices) == 0 :
            indices = np.argsort(mstLengths) 
            
            mstLengths = [mstLengths[i] for i in indices]
            # mstLengths = sorted(mstLengths)

            medianIdx = len(mstLengths)//2 
            medianLength = mstLengths[medianIdx]

            costOverTime = self.costOverTimes[i][medianLength]
            
            timing = list(costOverTime.keys())
            costs = list(costOverTime.values())

            # print(timing, costs)
            if len(costs) == 1 :
                 plt.axhline(y=costs[0], linestyle='--', color='red')
                 plt.plot(timing, costs, marker='o', markersize=4)
            else :
                plt.plot(timing, costs)

        plt.title(self.title)
        plt.ylabel("Solution Cost")
        plt.xlabel("Time")
        plt.legend(self.names)
        plt.tight_layout()
        plt.savefig("Plots/Comparisons/" + self.outputName + "_costOverTime.png", dpi=200)
        plt.close()

    def plotTSPMSTtogether(self):

        df = pd.DataFrame(columns=['Cost', 'Type', 'Planner'])
        df2 = pd.DataFrame(columns=['Path Cost', 'Type', 'Planner'])

        problem = "problem"
        if "Uniform" in self.env :
            problem = "Uniform HyperRectangles R^8"
        else :
            problem = "Center Obstacle R^8"

        names = list(self.names) 
        if names[1] == 'PRMbaseline' :
            names = ['IST*', 'Baseline']
        
        for i, name in enumerate(names):
            for j in range(len(self.algos[i])) :
                mstCost = self.algos[i][j] 
                tspCost = self.tspCosts[i][j] 

                col = {'Cost': mstCost, 'Type': 'Steiner Tree', 'Planner': name}
                df = df.append(col, ignore_index=True)
                
                col = {'Cost': tspCost, 'Type': 'Feasible Path', 'Planner': name}
                df = df.append(col, ignore_index=True)

                col = {'Path Cost': tspCost, 'Type': problem, 'Planner': name}
                df2 = df2.append(col, ignore_index=True)

        # print(df)
        colors = ['aquamarine', 'lightskyblue', 'lightgrey', 'lavender']
        boxplot = sns.boxplot(data=df, x='Type', y='Cost', hue='Planner', width=0.3, palette = colors, linewidth=0.5, fliersize=3, medianprops=dict(color='red'), flierprops =dict(color='red'))
        # plt.show()
        boxplot.set(xlabel=None)
        plt.title(self.title)
        plt.tight_layout()
        plt.savefig("Plots/Comparisons/" + self.outputName + "_TSPMST_plot.png", dpi=400)
        plt.close()
        return df2


    def plot(self):
        """ 
        fig, ax = plt.subplots()
        columns = [fixed_acidity, free_sulfur_dioxide, total_sulfur_dioxide, alcohol]
        fig, ax = plt.subplots()
        box = ax.boxplot(columns, notch=True, patch_artist=True)
        plt.xticks([1, 2, 3, 4], ["Fixed acidity", "Free sulfur dioxide", "Total sulfur dioxide", "Alochol"])

        colors = ['#0000FF', '#00FF00', '#FFFF00', '#FF00FF']

        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)

        plt.show()
        """
        self.fig, self.ax = plt.subplots()
        # columns = [self.sstar, self.sff, self.baseline, self.baselinePruned]
        columns = self.algos 

        # colors = ['#0000FF', '#00FF00', '#FFFF00', '#FF00FF']
        colors = ['aquamarine', 'lightskyblue', 'lightgrey', 'lavender']
        # box = self.ax.boxplot(columns, notch=True, patch_artist=True)
        box = self.ax.boxplot(columns, medianprops=dict(color='red'), patch_artist=True)
        # plt.axhline(y = self.lowerBound, color = 'g', linestyle = 'dashed', label = "Lower Bound")

        names = list(self.names) 
        if names[1] == 'PRMbaseline' :
            names = ['IST*', 'Baseline', 'SFF*', 'Multi-T RRT']

        plt.xticks(list(range(1, len(names) + 1)), names)
        # plt.xticks([1, 2, 3, 4], self.names)

        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)
        
        # plt.yscale('log', basey=2)
        plt.title(self.title)
        plt.ylabel("MST Cost")
        # plt.xlabel("Solver")
        plt.legend()
        plt.tight_layout()
        plt.show()
        # plt.savefig("Plots/Comparisons/" + self.outputName + "_plot.png", dpi=200)
        plt.close()

        # -----------------------------------------------------------------------------------------------------
        if not self.unsolved :
            return 
            
        self.fig2, self.ax2 = plt.subplots()
        graph = plt.bar(self.names, self.solved, color = self.colors)

        plt.yticks(list(range(0, self.runs + 1, 2)))
        # yticks = list(np.arange(1, self.runs + 1, 1))
        # print(yticks)
        # self.ax2.set_yticks(yticks)
        plt.ylabel("Runs")
        plt.title("Unsolved % of different solvers")

        i = 0 
        for p in graph:
            width = p.get_width()
            height = p.get_height()
            x, y = p.get_xy()
            plt.text(x+width/2,
                    y+height*1.01,
                    str(round(float(self.solved[i])*100.0/self.runs))+'%',
                    ha='center',
                    weight='bold')
            i+=1

        plt.tight_layout()
        plt.show()
        # plt.savefig("Plots/Comparisons/" + self.outputName + "_solved.png", dpi=200)
        plt.close()
        pass 

# Old Way:
# instances = []
# for instance in INSTANCES:
#     for factor in range(1, FACTOR+1):
#         instances.append((instance[0]*factor, instance[1]*factor, instance[2], instance[3]))


# New way:
instances = []
for instance in INSTANCES:
    j = 1
    for factor in range(1, 6, 2):
        instances.append((instance[0]*factor, instance[1], instance[2], instance[3]))
        j += 1 

# print(instances)



def bar_plot(diction, terminals = 10):
    fig, ax = plt.subplots()

    width = 0.18
    names = ['SFF*', 'IST*', 'Baseline', 'Multi-T RRT']
    bars = list(names)
    factors = [-1.5, -0.5, 0.5, 1.5]
    colors = ['lightskyblue', 'aquamarine', 'lightgrey', 'lavender']
    index = np.arange(len(mapping))
    environments = []
    for filename in diction :
        first = filename.find("_")
        prefix = filename[:first]
        environments.append(prefix)
    
    plt.xticks(index)
    solved = []

    for i, name in enumerate(names) :
        means = []
        errs = []
        solved.append([])

        for j, filename in enumerate(diction) :
            obj = diction[filename]
            lower_bound = obj.lowerBound 
            columns = obj.algos 
            if len(columns[i]) == 0 :
                mean, err = [0, 0]
                # plt.scatter(index[j] + factors[i] * width, 2, 2**8, marker = 'X')
            else :
                data = np.array(columns[i]) 
                # data = (data - lower_bound) / float(lower_bound)
                # data = data * 100.0 

                mean, err = mean_confidence_interval(data)
            solved[i].append(obj.solved[i]/float(obj.runs))
            means.append(mean)
            errs.append(err)
        
        # print(name, means, errs)
        bars[i] = ax.bar(index + factors[i] * width, means, width, yerr = errs, label = names[i], color = colors[i])
            
    # print('X-limits:', plt.xlim())
    lower_bounds = []
    xmins = []
    xmaxs = []
    for i, filename in enumerate(diction.keys()) : 
        obj = diction[filename]
        unsolved = np.array(obj.solved)
        unsolved = unsolved / int(obj.runs)
        unsolved = unsolved * 100 
        # print(unsolved)

        lower_bound = obj.lowerBound 
        mid = index[i] 
        lower_bounds.append(lower_bound)
        xmins.append(mid - width * 2)
        xmaxs.append(mid + width * 2)
    
        # plt.hlines(y = lower_bound, xmin = mid - width * 2, xmax = mid + width * 2, \
        #                 color = 'r', linestyle = 'dashed', label='Lower Bound')
        # print(lower_bound, mid - width, mid + width)
    
    plt.hlines(y = lower_bounds, xmin = xmins, xmax = xmaxs, \
                        color = 'r', linestyle = 'dashed', label='Lower Bound')

    plt.yscale('log', basey=2)
    # plt.ylim((0.1, 200))
    # plt.xticks(index, [10, 30, 50])

    min_height = plt.ylim()[0]
    
    for i, name in enumerate(names) :
        graph = bars[i] 
        
        for k, p in enumerate(graph):
            width = p.get_width()
            height = p.get_height()
            x, y = p.get_xy()
            here = float(solved[i][k])*100.0
            # print(x, y, height, here)
            if here < 0.5 :
                continue 

            if height < 2 :
                # print(height, here, index[k] + factors[i] * width)
                plt.scatter(index[k] + factors[i] * width * 1.05, min_height * 1.001, s = 165, marker = 'X', color='black')
                height = min_height
                continue 

            plt.text(x+width/2 - width * 0.2,
                    y+height*1.05,
                    str(round(here))+'%',
                    ha='center',
                    weight='bold', fontsize = 12)
    
    # plt.title(title)
    # plt.title("Terminals: " + str(num_terminals), fontsize = 15)
    plt.ylabel("MST Cost", fontsize = 15)
    # plt.xlabel("Solver")
    plt.yticks(fontsize=18)
    plt.xticks(index, environments, fontsize = 15)
    plt.legend(fontsize = 14)
    plt.tight_layout()
    # plt.show()
    plt.savefig("Plots/Comparisons/" + str(terminals) + "_terminals_all.png", dpi=300)
    plt.close()

num_terminals = 50
directory = 'Logx/Others/Prefinal logs'
mapping = {}
to_search = "_" + str(num_terminals) + "_"

dirs = os.listdir(directory)
filtered = [filename for filename in dirs if filename.endswith('.xml') and to_search in filename]
filtered[-1], filtered[-3] = filtered[-3], filtered[-1]
filtered[-1], filtered[-2] = filtered[-2], filtered[-1]


for filename in filtered :
    wholePath = directory + "/" + filename 
    obj = OutputPlotter(wholePath)
    obj.parseOutputFile()
    mapping[filename] = obj 
    # obj.plot()
    # obj.bar_plot()
        
bar_plot(mapping, num_terminals)

# for instance in instances:
#     numTerminals, timeLimit, envPath, robotPath = instance
#     outFile = generateLogName(numTerminals, envPath, timeLimit, "Logx/Prefinal logs")
#     print(outFile)
#     obj = OutputPlotter(outFile)
#     obj.parseOutputFile()

    # obj.plot()
    # obj.plotOverTime()
    # obj.plotDistributionOverTime()
    
    # obj.plotTSPcosts()
    # df = obj.plotTSPMSTtogether()
    # dataframes.append(df)

    # break 

# plotTSPs(dataframes[0], dataframes[1])

# Plot pruning 
# for instance in INSTANCES:
#     numTerminals, timeLimit, envPath, robotPath = instance
#     outFile = generateLogName(numTerminals, envPath, timeLimit, "Logx/PruningStats")
#     print(outFile)
#     obj = OutputPlotter(outFile)
#     obj.parseOutputFile()
#     obj.getPruning()

# obj.plot()