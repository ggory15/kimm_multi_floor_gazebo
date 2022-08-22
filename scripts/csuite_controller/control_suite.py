import os, time, sys
import numpy as np

import importlib, pkgutil
import yaml
import copy
from .gazebo_suite import GazeboInterface
from .rviz_suite import RvizInterface

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuite():
    def __init__(self):
        self.gazebo = []
        self.rviz = []
        self.enableGZ = False
        self.enableRviz = False

    def launchgz(self, stage):
        self.enableGZ = True
        self.gazebo = GazeboInterface(stage)
    
    def launchrviz(self):
        self.enableRviz = True
        self.rviz = RvizInterface()
