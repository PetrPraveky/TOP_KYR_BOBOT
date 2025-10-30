import numpy as np
from ctu_crs import CRS97
# from ctu_crs import CRS93
import cv2
from perception import *
from aruco_detection import *

# Initialize the robot interface
robot = CRS97()
# robot = CRS93()

input("Arm the arm!")

robot.initialize(home=False)

robot.wait_for_motion_stop()

robot.close()