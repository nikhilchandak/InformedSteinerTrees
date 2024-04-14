# from xml.dom import minidom
# root = minidom.Document()
  
# problem = root.createElement('Problem') 
# root.appendChild(problem)
  
# productChild = root.createElement('product')
# productChild.setAttribute('name', 'Geeks for Geeks')
  
# child = root.createElement("whatever")
# productChild.appendChild(child)
# problem.appendChild(productChild)
  
# xml_str = root.toprettyxml(indent ="\t") 
# print(xml_str)

# import subprocess
# res = subprocess.run(["./SFF/sffstar", "./SFF/test_2D.xml"], capture_output=True, text=True)
# print(res.stdout)


######### PROFILING #################

# import matplotlib.pyplot as plt 

# xaxis = ['growPRM', 'expandPRM', 'S*', 'InformedSampling', 'Others']
# # yaxis = [33, 14, 44, 0.5, 8.5]
# yaxis = [40.5, 4.5, 52, 0.5, 2.5]

# xaxis = ['grow\nPRM', 'expand\nPRM', 'S*', 'Informed\nSampling', 'CopyPRM\nGraph', 'Lemma,\n MST & PDF', 'MapOMPL\nTerminals']
# yaxis = [68.6, 7.6, 14.2, 1.1, 5.0, 2.5, 1]

# plt.rc('xtick', labelsize=9) 

# fig3, ax3 = plt.subplots()
# graph = plt.bar(xaxis, yaxis)

# # y_pos = range(len(xaxis))
# # plt.xticks(y_pos, xaxis, rotation=90)

# plt.ylabel("Time taken (in %)")
# # plt.title("Runtime breakdown of Major Components in Naive S*")
# plt.title("Runtime breakdown of Major Components in Incremental Ripple")

# i = 0 
# for p in graph:
#     width = p.get_width()
#     height = p.get_height()
#     x, y = p.get_xy()
#     plt.text(x+width/2,
#             y+height*1.01,
#             str(yaxis[i])+'%',
#             ha='center',
#             weight='bold',
#             wrap = False) 
#             # va = 'top')
#     i+=1

# plt.tight_layout()
# # plt.savefig("NaiveSstarTimeBreakdown.png", dpi=200, bbox_inches='tight')
# plt.savefig("IncrementalRippleTimeBreakdown2.png", dpi=200, bbox_inches='tight')
# plt.close()

################ PROFILING ENDS #####################

# META - ANALYSIS

# ############ INCREMENTAL ########################
# # grow, expand, s*, plannerDataCopy, informedSampling, 
# # (lemma, probability distribution, update MST)
# # check Terminals have been added so as to map OMPL vertices
# # with terminal indices
# x = [91.5 * 0.9, 91.5 * 0.1, 17, 6, 1.2, 3, 1.3]

# print(x)
# z = sum(x)
# print(z)

# for i in range(len(x)):
#     x[i] *= 100.0
#     x[i] /= float(z)
#     x[i] = round(x[i], 2)

# print(x, sum(x))

# print("--------------")

# ############## NAIVE S* ########################
# # s*, grow, expand, informed sampling, others 
# y = [69, 58 * 0.9, 58 * 0.1, 3, 1]

# print()
# print(y)
# z = sum(y)
# print(z)

# for i in range(len(y)):
#     y[i] *= 100.0
#     y[i] /= float(z)
#     y[i] = round(y[i], 2)
    
# print()
# print(y, sum(y))



############# RANDOM WORLD ################


import matplotlib.pyplot as plt 
import numpy as np 
from matplotlib.patches import Rectangle

fig, ax = plt.subplots()

with open('dummy.txt', 'r') as f :
    n = int(f.readline())
    
    for i in range(n):
        x1, y1 = list(map(float, f.readline().split()))
        x2, y2 = list(map(float, f.readline().split()))
        
        x1 = min(1, x1) ; x2 = min(1, x2) ; y1 = min(1, y1); y2 = min(1, y2)
        x1 = max(0, x1) ; x2 = max(0, x2) ; y1 = max(0, y1) ; y2 = max(0, y2) ;

        # add rectangle to plot
        w = x2 - x1 
        h = y2 - y1
        ax.add_patch(Rectangle((x1, y1), w, h, facecolor='black', alpha=0.1))
        
        # plt.pause(2)
        # print(i, (x1, y1), (w, h))

#display plot
plt.show()