# Informed Steiner Trees (IST*) for Multi-Goal Motion Planning 

## Required Packages 

Matplotlib, Scipy, Numpy, OMPL, Shapely, elkai (for TSP tour computation)

Install most of them by the following command:

```
pip3 install -r requirements.txt
```

## Dependencies (or other relevant repositories):

### Needs to be installed manually:

- [Open Motion Planning Library](https://github.com/ompl/ompl) with **Python bindings**.
- [Steinerpy](https://github.com/Kchour/steinerpy) -- Codebase for [S*: A Heuristic Information-Based Approximation Framework for Multi-Goal Path Finding](https://ojs.aaai.org/index.php/ICAPS/article/view/15950) whose installation instructions can be found at its github homepage. Best way is to install it in developer mode.

### Already part of this repository (pre-imported):

- [Space Filling Forest*](https://github.com/ctu-mrs/space_filling_forest_star) -- Codebase for [Multi-goal path planning using multiple random trees](https://ieeexplore.ieee.org/abstract/document/9385932)

## Main Files 

-`modularPRM.py` : Contains all the code relevant to PRM, pruning, baseline, informed sampling, loading environment, etc.

-`plannerGraph.py` : Contains code related to running S* on the PRM graph. 

-`config.py`: Declare path to environment/robot files and create instances (give time (in seconds for each execution), number of terminals to env/robot) which may be later used for benchmarking/plotting. 

-`benchmarking.py`: **Parellely** runs all the algorithms (SBS*/SFF/Baseline/UninformedSBS*/etc.) on the instances declared in config (single benchmark may be done by calling `run(...)`) via multithreading. 

## Misc

- Run virtual environment (I'm using 3.7). Also make sure to switch to 3.7 via `sudo update-alternatives --config python3`
- Install python dependencies from requirements.txt
- Make sure ompl is installed with python bindings. Make a copy of ompl's python installation (`/usr/lib/python3/ompl`) in current active python (3.7)
- Add ompl path to pythonpath via `source ompl_path.sh`
- Install [steinerpy](https://github.com/Kchour/steinerpy) in edittable mode for developer use (python3 -m pip install -e path_to_steinerpy_repo) 

## Citation 

If you use this repository in your research, you are encouraged to cite our work:

``` 
@article{chandak2023informed,
  title={Informed steiner trees: Sampling and pruning for multi-goal path finding in high dimensions},
  author={Chandak, Nikhil and Chour, Kenny and Rathinam, Sivakumar and Ravi, Ramamoorthi},
  journal={IEEE Transactions on Automation Science and Engineering},
  year={2023},
  publisher={IEEE}
}
```
