import numpy as np
from ctu_crs import CRS97
# from ctu_crs import CRS93
import cv2
from perception import *
from aruco_detection import *
from Controller import *


# Initialize the robot interface
robot = CRS97()
# robot = CRS93()

#test = input("zapni ruku")
robot.initialize(home=False)

master = Controller(robot)

while(True):
    raw_input = input("Enter your instruction:\n")

    master.handle_input(raw_input)