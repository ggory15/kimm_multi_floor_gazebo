import subprocess, signal, os, inspect, time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np
from std_msgs.msg import String, Float32, Int16, Bool

from copy import deepcopy 
import std_msgs.msg
import geometry_msgs.msg


from visualization_msgs.msg import Marker, MarkerArray

class RvizInterface():
    def __init__(self):
        self.devnull = open(os.devnull, 'wb')
        self.rviz_iface = subprocess.Popen("ros2 launch kimm_multi_floor_gazebo multi_floor_navigation_launch.py",
                                        stdout=subprocess.PIPE,
                                        shell=True,
                                        preexec_fn=os.setsid,
                                        stderr=self.devnull)
    def __del__(self):
        os.killpg(os.getpgid(self.rviz_iface.pid), signal.SIGINT)
    