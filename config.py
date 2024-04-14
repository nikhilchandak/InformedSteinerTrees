"""Import this module to access global settings. Can be used
    to configure behavior

"""

# import logging, logging.config
from multiprocessing import current_process
from collections import defaultdict
import threading

############################################################
# Important constants below
############################################################
SEED = 12345

# goalCount = 6
EDGE_TIME_LIMIT = 0.05
EPS = 1e-6

edgeNumberMap = {}
ndim = 2 

RANDOM_POLYGONS_ENV = '2D/RandomPolygons_planar_env.dae'
BUG_TRAP_PLANNER2D_ENV = '2D/BugTrap_planar_env.dae'
MAZE_PLANNER_ENV = '2D/Maze_planar_env.dae'

BARRIERS_EASY_ENV = '2D/Barriers_easy_env.dae'
BARRIERS_EASY_ROBOT = '2D/Barriers_easy_robot.dae'

UNIQUE_MAZE_ENV = '2D/UniqueSolutionMaze_env.dae'
UNIQUE_MAZE_ROBOT = '2D/UniqueSolutionMaze_robot.dae'

APARTMENT_ENV = '3D/Apartment_env.dae'
APARTMENT_ROBOT = '3D/Apartment_robot.dae'

ABSTRACT_ENV = '3D/Abstract_env.dae'
ABSTRACT_ROBOT = '3D/Abstract_robot.dae'

CUBICLES_ENV = '3D/cubicles_env.dae'
CUBICLES_ROBOT = '3D/cubicles_robot.dae'

STRAIGHT_ROBOT = '2D/StraightC_planar_robot.dae'
CAR2_ROBOT = '2D/car2_planar_robot.dae'

HOME_ENV = '3D/Home_env.dae'
HOME_ROBOT = '3D/Home_robot.dae'

TWISTY_COOL_ENV = '3D/Twistycool_env.dae'
TWISTY_COOL_ROBOT = '3D/Twistycool_robot.dae'

TWISTY_COOLER_ENV = '3D/Twistycooler_env.dae'
TWISTY_COOLER_ROBOT = '3D/Twistycooler_robot.dae'

# From SFF* : 
BUILDING_ENV = 'SF/building.dae'
ROBOT_CYLINDER_SMALL = 'SF/robot_cylinder_small.dae'
TRIANG_ENV = 'SF/triang.dae'
DENSE_ENV = 'SF/dense_3D.dae'
ROBOT_SMALL = 'SF/robot_small.dae'

# For R^n
HYPERCUBE_EDGES = "4D/HypercubeEdgesR4.dae"

RANDOM_WORLD = "8D/RandomWorldR8.dae"
RANDOM_WORLD_R8 = "8D/RandomWorldR8.dae"
RANDOM_WORLD_R4 = "4D/RandomWorldR4.dae"

CENTER_OBSTACLE = "4D/CenterObstacleR4.dae"
CENTER_OBSTACLE_R8 = "8D/CenterObstacleR8.dae"
CENTER_OBSTACLE_R4 = "4D/CenterObstacleR4.dae"

HYPER_RECTANGLES = "8D/UniformHyperRectanglesR8.dae"
HYPER_RECTANGLES_R8 = "8D/UniformHyperRectanglesR8.dae"
HYPER_RECTANGLES_R4 = "4D/UniformHyperRectanglesR4.dae"

POINT_ROBOT = "0D/PointRobot.dae"

FACTOR = 5
# INSTANCES = [(10, 120, RANDOM_POLYGONS_ENV, CAR2_ROBOT), (10, 120, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT), (10, 120, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT)]

INSTANCES = [(10, 120, RANDOM_POLYGONS_ENV, CAR2_ROBOT), (10, 120, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT)]

INSTANCES = [(10, 600, HOME_ENV, HOME_ROBOT)] #, (10, 600, APARTMENT_ENV, APARTMENT_ROBOT)] #, 

# INSTANCES = [(10, 600, ABSTRACT_ENV, ABSTRACT_ROBOT)]

# # INSTANCES = [(10, 600, TWISTY_COOLER_ENV, TWISTY_COOLER_ROBOT)]
# INSTANCES = [(10, 600, TWISTY_COOL_ENV, TWISTY_COOL_ROBOT)]
 
# INSTANCES = [(10, 600, HOME_ENV, HOME_ROBOT)]
# INSTANCES = [ (10, 60, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT), (10, 120, ABSTRACT_ENV, ABSTRACT_ROBOT) ]
# INSTANCES = [ (10, 300, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT)]
INSTANCES = [ (10, 120, ABSTRACT_ENV, ABSTRACT_ROBOT) ]
# INSTANCES = [ (10, 180, BUG_TRAP_PLANNER2D_ENV, CAR2_ROBOT) ]
# INSTANCES = [ (10, 600, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT) ]
# INSTANCES = [ (10, 450, APARTMENT_ENV, APARTMENT_ROBOT) ]
# INSTANCES = [ (10, 450, CUBICLES_ENV, CUBICLES_ROBOT) ]

# INSTANCES = [ (10, 30, MAZE_PLANNER_ENV, CAR2_ROBOT)]

#(10, 900, TRIANG_ENV, ROBOT_CYLINDER_SMALL), (10, 1200, BUILDING_ENV, ROBOT_CYLINDER_SMALL)

# INSTANCES = [(10, 3600, ABSTRACT_ENV, ABSTRACT_ROBOT), (10, 3600, HOME_ENV, HOME_ROBOT) ]

# INSTANCES = [(10, 240, BUILDING_ENV, ROBOT_CYLINDER_SMALL)]
# ORDERS = [(10, 60, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT), (10, 60, BUG_TRAP_PLANNER2D_ENV, CAR2_ROBOT)]
# ORDERS = [(10, 240, TRIANG_ENV, ROBOT_CYLINDER_SMALL)]


# INSTANCES = [(10, 60, MAZE_PLANNER_ENV, CAR2_ROBOT), (10, 60, BUG_TRAP_PLANNER2D_ENV, CAR2_ROBOT)]
# INSTANCES = [(10, 900, DENSE_ENV, ROBOT_CYLINDER_SMALL)]
# INSTANCES = [(10, 300, TRIANG_ENV, ROBOT_CYLINDER_SMALL)]

# ORDERS = [(10, 1200, HYPER_RECTANGLES_R8, POINT_ROBOT)]
ORDERS = [(10, 600, HYPER_RECTANGLES_R4, POINT_ROBOT)]
# ORDERS = [(10, 300, HYPERCUBE_EDGES, POINT_ROBOT)]
ORDERS = [(10, 450, CENTER_OBSTACLE_R4, POINT_ROBOT)]
# ORDERS = [(10, 900, CENTER_OBSTACLE, POINT_ROBOT)]
# ORDERS = [(10, 300, RANDOM_WORLD, POINT_ROBOT)]
# ORDERS = [(50, 3600, HYPER_RECTANGLES_R8, POINT_ROBOT)]
ORDERS = [(50, 2700, CENTER_OBSTACLE_R8, POINT_ROBOT)]
# ORDERS = [(10, 300, CENTER_OBSTACLE_R4, POINT_ROBOT)]

# ALL = [(10, 600, HYPER_RECTANGLES, POINT_ROBOT)]
# ALL =  [(10, 300, HYPERCUBE_EDGES, POINT_ROBOT)]
# ALL = [(10, 450, CENTER_OBSTACLE, POINT_ROBOT)]
# ALL = [(10, 300, TRIANG_ENV, ROBOT_CYLINDER_SMALL), (10, 600, HOME_ENV, HOME_ROBOT), (10, 300, ABSTRACT_ENV, ABSTRACT_ROBOT), (10, 180, ABSTRACT_ENV, ABSTRACT_ROBOT)]
# INSTANCES = ALL 

INSTANCES = ORDERS 

INSTANCES = [(50, 3600, HYPER_RECTANGLES_R8, POINT_ROBOT), (50, 2700, CENTER_OBSTACLE_R8, POINT_ROBOT)]

# INSTANCES = [(10, 120, RANDOM_POLYGONS_ENV, CAR2_ROBOT), (10, 120, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT)]
# INSTANCES = [(10, 240, ABSTRACT_ENV, ABSTRACT_ROBOT)]
# INSTANCES = [(10, 180, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT)]

# INCREMENTAL_SUITE = [(10, 60, MAZE_PLANNER_ENV, CAR2_ROBOT), (10, 120, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT), (10, 240, ABSTRACT_ENV, ABSTRACT_ROBOT)]
# INCREMENTAL_SUITE = [(10, 300, HOME_ENV, HOME_ROBOT), (10, 240, TRIANG_ENV, ROBOT_CYLINDER_SMALL)]
# INSTANCES = [(10, 400, HYPER_RECTANGLES, POINT_ROBOT)]
# INSTANCES = INCREMENTAL_SUITE 

# PRUNING_STATS = [(10, 240, ABSTRACT_ENV, ABSTRACT_ROBOT),  (10, 240, HOME_ENV, HOME_ROBOT), (10, 120, MAZE_PLANNER_ENV, CAR2_ROBOT), (10, 120, BUG_TRAP_PLANNER2D_ENV, CAR2_ROBOT), (10, 120, RANDOM_POLYGONS_ENV, CAR2_ROBOT), (10, 120, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT), (10, 120, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT)]
# INSTANCES = PRUNING_STATS

# SAMPLING_STATS = [ (10, 240, ABSTRACT_ENV, ABSTRACT_ROBOT), (10, 120, BARRIERS_EASY_ENV, BARRIERS_EASY_ROBOT),  (10, 600, HOME_ENV, HOME_ROBOT), (10, 180, MAZE_PLANNER_ENV, CAR2_ROBOT), (10, 180, UNIQUE_MAZE_ENV, UNIQUE_MAZE_ROBOT), (10, 300, CENTER_OBSTACLE, POINT_ROBOT)]
# INSTANCES = SAMPLING_STATS

RESULTS_EXCEPT_ABSTRACT = [(10, 1200, HYPER_RECTANGLES_R8, POINT_ROBOT), 
                (10, 600, HYPER_RECTANGLES_R4, POINT_ROBOT), 
                (10, 900, CENTER_OBSTACLE_R8, POINT_ROBOT), 
                (10, 450, CENTER_OBSTACLE_R4, POINT_ROBOT),
                (10, 600, HOME_ENV, HOME_ROBOT), 
                (10, 600, TWISTY_COOL_ENV, TWISTY_COOL_ROBOT) ]

ABSTRACT_PROBELM = [(10, 600, ABSTRACT_ENV, ABSTRACT_ROBOT)]

INSTANCES = ABSTRACT_PROBELM
# INSTANCES = [(30, 900, DENSE_ENV, ROBOT_CYLINDER_SMALL)]
# INSTANCES = [(10, 30, TRIANG_ENV, ROBOT_CYLINDER_SMALL)]
# INSTANCES = [(5, 600, BUILDING_ENV, ROBOT_CYLINDER_SMALL)]
# INSTANCES = [(30, 1200, HOME_ENV, HOME_ROBOT)] 
# INSTANCES = RESULTS_EXCEPT_ABSTRACT

#######################################################################
# Logging related
#######################################################################

# USAGE:
#
# ```
# import logging
# from config import UserConfigLogs

# my_logger = logging.getLogger(__name__)

# ...

# def your_function():
#     my_logger.debug("insert string message here", exc_info=True)
#     my_logger.info("insert string message here", exc_info=True)
#     my_logger.warn("insert string message here", exc_info=True)
#     my_logger.error("insert string message here", exc_info=True)
#     my_logger.critical("insert string message here", exc_info=True)


# ```
# exc_info is used to record exceptions

# The following two custom classes is used to solve
# issues related to logging in multithreaded applications
# where locks() are reused!
# class _ProcessSafeStreamHandler(logging.StreamHandler):
#     def __init__(self):
#         super().__init__()

#         self._locks = defaultdict(lambda: threading.RLock())

#     def acquire(self):
#         current_process_id = current_process().pid
#         self._locks[current_process_id].acquire()

#     def release(self):
#         current_process_id = current_process().pid
#         self._locks[current_process_id].release()

# class _ProcessSafeFileHandler(logging.FileHandler):
#     def __init__(self, *args, **kwargs):
#         super().__init__(*args, **kwargs)

#         self._locks = defaultdict(lambda: threading.RLock())

#     def acquire(self):
#         current_process_id = current_process().pid
#         self._locks[current_process_id].acquire()

#     def release(self):
#         current_process_id = current_process().pid
#         self._locks[current_process_id].release()


# class _ColoredFormatter(logging.Formatter):
#     """Customized custom formatter class to allow colored outputs to the console!
    
#     """
#     # logging colors
#     DEBUG = "\033[92m"  # LIGHT GREEN
#     INFO = "\033[94m"   # LIGHT BLUE
#     # DEBUG = '\033[32m'    # GREEN
#     # INFO = "\033[97m"     # BLUE
#     WARNING = "\033[33m"    # YELLOW
#     ERROR = "\033[31m"   # RED
#     CRITICAL = '\033[91m'      # High-intensity RED

#     HEADER = '\033[95m'
#     OKBLUE = '\033[94m'
#     WARNING = '\033[93m'
#     ENDC = '\033[0m'        # RESET
#     BOLD = '\033[1m'
#     UNDERLINE = '\033[4m'

#     # output format
#     format = "%(asctime)-15s : %(levelname)-4s : %(name)-15s : %(message)s"

#     # new format map
#     COLOR_FMT = {
#         'WARNING': "".join((WARNING, format, ENDC)),
#         'INFO': "".join((INFO, format, ENDC)),
#         'DEBUG': "".join((DEBUG, format, ENDC)),
#         'CRITICAL': "".join((CRITICAL, format, ENDC)),
#         'ERROR': "".join((ERROR, format, ENDC)),
#     }

#     def format(self, record):
#         """override format method"""
#         # get name and level of recording
#         levelname= record.levelname
#         levelno = record.levelno

#         # get new format from map
#         new_fmt = _ColoredFormatter.COLOR_FMT[levelname]

#         # create a new formatter object
#         formatter = logging.Formatter(new_fmt)

#         # return it wrapped around record!
#         return formatter.format(record)


# class UserConfigLogs:

#     # This can be configured by the user!
#     log_conf = {
#         'version': 1,
#         'disable_existing_loggers': True,
#         'formatters': {
#             'default': {
#                 'format': '%(asctime)-15s %(levelname)-8s %(name)-8s %(message)s'
#             },
#             'colored':{
#                 # https://docs.python.org/3/library/logging.config.html#dictionary-schema-details
#                 # alternatively, fastAGC.config.ColoredFormatter
#                 '()':  _ColoredFormatter
#             },
#         },
#         'handlers': {
#             'console': {
#                 '()': _ProcessSafeStreamHandler,
#                 'formatter': 'colored',
#                 'level': 'DEBUG'
#             },
#             'file_handler': {
#                 '()': _ProcessSafeFileHandler,
#                 'formatter': 'default',
#                 'level': 'DEBUG',
#                 'filename': '/tmp/sampling_sstar_logfile.log'
#             },
#         },
#         'loggers': {
#             # You can create a name of logger you desire
#             '': {# root logger
#                 'handlers': ['file_handler', 'console'],
#                 'level': 'DEBUG'
#             },

#         },
#     }
#     @classmethod
#     def load_log_conf(cls):
#         """USER MUST CALL THIS IF log_conf is changed!

#         """
#         logging.config.dictConfig(cls.log_conf)

# # load log conf upon first import
# UserConfigLogs.load_log_conf()