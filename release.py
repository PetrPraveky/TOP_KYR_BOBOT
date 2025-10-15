import numpy as np
from ctu_crs import CRS97
import cv2
from perception import *

# Initialize the robot interface
robot = CRS97()
robot.release()