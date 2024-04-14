"""WIP
"""

import unittest
import random 

random.seed(1234)

from incrementalSstar import IncrementalSstarVirtual, \
                                IncrementalSstar2D_KDTree,\
                                 RandomTestGraph, Utils

from steinerpy.library.animation import AnimateV2

# class IncrementalSstarKDTree(IncrementalSstarVirtual):



class IncrementalSstarTest(unittest.TestCase):


    def test_kd_tree_virtual_class(self):
        # bounding box
        minX, maxX, minY, maxY = [-10, 10, -10, 10]
        # number of terminals
        num_terminals = 5
        # number of nodes in the graph
        num_of_nodes = 10

        # create a random 2d graph
        rtg = RandomTestGraph(num_of_nodes=num_of_nodes, minX=minX, maxX=maxX, minY=minY, maxY=maxY)
        
        # obtain a random subset of terminals from the graph
        terminals = Utils.random_subset(rtg.nodes(), max_num_items=num_terminals)

        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        fig, ax = plt.subplots(figsize=(7,7))
        ax.set_aspect("equal")
        ax.set_xlim(minX, maxX)
        ax.set_ylim(minY, maxY)
        line_segments = LineCollection(list(rtg.edges()), colors="gray", alpha=1)
        ax.add_collection(line_segments)
        # ax.autoscale()

        AnimateV2.init_figure(fig, ax)
        
        # Try our hand at Incremental S*
        isv = IncrementalSstar2D_KDTree(rtg, terminals, variant="S*-BS")

        # try adding random points
        samples = Utils.random_sampler(1000, minX, maxX, minY, maxY)
        # samples = Utils.random_sampler(10000, -10.0, -9.5, 5.6, 8.3)
        for s in samples:
            isv.add(s)

        # plt.show(block=True)

if __name__ == "__main__":
    unittest.main()