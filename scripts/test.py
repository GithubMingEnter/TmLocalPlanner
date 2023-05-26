# coding=utf-8‘
from pdb import runcall
from sys import executable
import time
import atexit
from numpy import dtype
import rospy
import copy
import tf
import numpy as np
from scipy import spatial
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from pyomo.environ import *
from pyomo.dae import *
from scipy.interpolate import interp1d
import matplotlib
import matplotlib.pyplot as plt
from ackermann_msgs.msg import AckermannDrive
import math
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import *  # 与gazebo交互的server头文件
from geometry_msgs.msg import Pose
from time import sleep


x = [1, 2, 3]
print(len(x))
data = Path()
path_array = data.poses

print(len(path_array))



