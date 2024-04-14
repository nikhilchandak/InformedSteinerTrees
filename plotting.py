import xml.etree.ElementTree as ET
from xml.dom import minidom

from matplotlib import pyplot as plt
from benchmarking import generateLogName
import numpy as np
import scipy.stats as st
from config import * 
import sys 

import pandas as pd
import seaborn as sns

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

        self.numAlgos = 2 
        self.names = ["Ripple", "PRMbaseline"]

        if "Sstar" in outputFilePath :
            self.names = ["Ripple", "Sstar"]

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

    def getPruning(self, folder="Logx/BaselinePRM/"):
        env = self.env[3:-4]
        self.pruning = []
        terminalsList = []
        totalEdges = []
        maximums = []

        j = 0
        for factor in range(1, FACTOR+1, 2):
            j += 1 
            cnt = 0.0 
            prunedTotal = 0 
            fileName = env + "_" + str(self.numTerminals * factor) + "_" + str(self.timeLimit * j) + "s.xml"
            
            maxi = 0

            # print(fileName)
            doc = minidom.parse(folder + fileName)
            outputs = doc.getElementsByTagName("Output")
            for output in outputs :
                try :
                    pruned = int(output.attributes['pruned'].value)
                    # print(pruned)
                    if pruned > 0 :
                        prunedTotal += pruned 
                        cnt += 1 
                        maxi = max(maxi, pruned)
                except :
                    pass
                # print(output.attributes.items())
                # sys.exit()
            if cnt != 0 :
                self.pruning.append(prunedTotal/cnt)
                # self.pruning.append(maxi)
                maximums.append(maxi)

                terminalsList.append(self.numTerminals * factor)
                # Nc2 - (N - 1)
                edgeCount = ( terminalsList[-1] * ( (terminalsList[-1] - 1) * 0.5) ) - (terminalsList[-1] - 1)
                totalEdges.append( edgeCount )

        # print(self.pruning)
        
        self.fig3, self.ax3 = plt.subplots()
        
        graph = plt.bar(terminalsList, totalEdges, width=7)
        plt.bar(terminalsList, self.pruning, width=7)
        plt.yscale('log', basey=2)

        # extraticks = [2**11]
        # self.fig3.canvas.draw()
        # curticks = list(self.ax3.get_yticks())
        # print(curticks)
        # plt.yticks(curticks)

        plt.legend(["#TotalEdges", "#PrunedEdges"])

        plt.ylabel("No. of edges pruned (skipped forever)")
        plt.xticks(terminalsList)
        plt.title("Edge Pruning Statistics: " + env)

        i = 0 
        for p in graph:
            width = p.get_width()
            height = p.get_height()
            x, y = p.get_xy()
            plt.text(x+width/2,
                    y+height*1.01,
                    "Avg: " + str(round(float(self.pruning[i])*100.0/totalEdges[i]))+'%' + "\n" + "Max: " + str(round(float(maximums[i])*100.0/totalEdges[i]))+'%',
                    ha='center',
                    weight='bold',
                    wrap = False) 
                    # va = 'top')
            i+=1

        plt.tight_layout()
        # plt.subplots_adjust(top=.9)
        plt.savefig(folder + env + "_pruned.png", dpi=200, bbox_inches='tight')
        plt.close()
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
            algo = self.doc.getElementsByTagName(self.names[i])[0]
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

                try :
                    self.averagePruned += int(output.attributes['pruned'].value)
                    self.ellipsesRemaining += int(output.attributes['ellipsesRemaining'].value)
                except :
                    pass 
            
            if unsolved > 0:
                self.unsolved = True 

            self.solved.append(unsolved)


        if self.unsolved != self.runs :
            self.averagePruned /= float(self.runs - self.unsolved)
            self.ellipsesRemaining /= float(self.runs - self.unsolved)


        edgeCount = ( self.numTerminals * ( (self.numTerminals - 1) * 0.5) ) - (self.numTerminals - 1)
        self.averagePruningPercent = int(self.averagePruned * 100.0 / edgeCount)

        self.title = str(self.env[3:-4]) + ":  Terminals = " + str(self.numTerminals) + "  Time = " + str(self.timeLimit) + "s" + "  Runs = " + str(self.runs)
        
        if self.averagePruned > 0 : 
            self.title += "\n Avg. Pruned = " + str(self.averagePruningPercent) + "%"

        print("Average Prune %:", self.averagePruningPercent, " #Pruned = ", self.averagePruned, " #EllipsesRemaining = ", self.ellipsesRemaining)
        print()
        pass 


    def plotDistributionOverTime(self):
        self.fig6, self.ax6 = plt.subplots()
        indices = []
        legendAxes = []

        def mean_confidence_interval(data, confidence=0.95):
            a = 1.0 * np.array(data)
            n = len(a)
            m, se = np.mean(a), st.sem(a)
            h = se * st.t.ppf((1 + confidence) / 2., n-1)
            return m, m-h, m+h

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

        # plt.ylabel("Solution Cost")
        # plt.xlabel("Time (in secs)")
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        # plt.legend(self.names, prop={'size': 12})
        names = list(self.names) 
        if names[1] == 'PRMbaseline' :
            names = ['IST*', 'Baseline']

        plt.legend(legendAxes, names, prop={'size': 20})

        
        xticks = plt.gca().get_xticklabels() # plt.xticks()[1]
        xticklabels = [str(label.get_text()) for label in xticks]

        # get the y tick labels
        yticks = plt.yticks()[1]
        yticklabels = [str(label.get_text()) for label in yticks]

        # print the tick labels
        # print("X tick labels:", xticklabels)
        # print("Y tick labels:", yticklabels)

        # plt.locator_params(axis='y', nbins=6)
        self.ax6.yaxis.set_major_locator(plt.MaxNLocator(7))
        self.ax6.xaxis.set_major_locator(plt.MaxNLocator(6))

        plt.tight_layout()
        plt.savefig("Plots/RevisedResults/" + self.outputName + "_costDistributionOverTime.png", dpi=400)
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
        plt.savefig("Plots/RevisedResults/" + self.outputName + "_costOverTime.png", dpi=200)
        plt.close()

    def plotTSPcosts(self):
        """ 
        Box plot of path costs.
        """
        self.fig, self.ax = plt.subplots()

        # box = self.ax.boxplot(self.tspCosts, patch_artist=True)
        colors = ['#0000FF', '#00FF00', '#FFFF00', '#FF00FF']
        colors = ['aquamarine', 'lightskyblue', 'lightgrey', 'lavender']

        box = self.ax.boxplot(self.tspCosts, medianprops=dict(color='red'), patch_artist=True)
        # plt.axhline(y = self.lowerBound, color = 'g', linestyle = 'dashed', label = "Lower Bound")

        names = list(self.names) 
        if names[1] == 'PRMbaseline' :
            names = ['IST*', 'Baseline']
        plt.xticks(list(range(1, len(self.names) + 1)), names)

        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)
        
        plt.title(self.title)
        plt.ylabel("Path Cost")
        plt.legend()
        plt.tight_layout()
        plt.savefig("Plots/TSP/" + self.outputName + "_Path_plot.png", dpi=400)
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
        plt.savefig("Plots/TSP_MST/" + self.outputName + "_TSPMST_plot.png", dpi=400)
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
            names = ['IST*', 'Baseline']
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
        plt.savefig("Plots/TSP/" + self.outputName + "_plot.png", dpi=200)
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
        plt.savefig("Plots/RevisedResults/" + self.outputName + "_solved.png", dpi=200)
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

def plotTSPs(df1, df2):
    print(df1)
    print(df2)
    df = pd.concat([df1, df2])
    print(df)

    
    colors = ['aquamarine', 'lightskyblue', 'lightgrey', 'lavender']
    boxplot = sns.boxplot(data=df, x='Type', y='Path Cost', hue='Planner', width=0.3, palette = colors, linewidth=0.5, fliersize=3, medianprops=dict(color='red'), flierprops =dict(color='red'))
    # plt.show()
    boxplot.set(xlabel=None)
    plt.title('')
    plt.tight_layout()
    plt.savefig("Plots/TSP_MST/" + "Combined_TSPMST_plot.png", dpi=500)
    plt.close()
    pass 

# Plot individual plots (solution cost, solved %)
dataframes = []

for instance in instances:
    numTerminals, timeLimit, envPath, robotPath = instance
    outFile = generateLogName(numTerminals, envPath, timeLimit, "Logx/PrefinalResults")
    print(outFile)
    obj = OutputPlotter(outFile)
    obj.parseOutputFile()

    # obj.plot()
    # obj.plotOverTime()
    obj.plotDistributionOverTime()
    
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