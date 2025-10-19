import numpy as np
from ctu_crs import CRS97
import cv2
from perception import *
from aruco_detection import *

MAX_ITERATIONS = 2
robot = CRS97()

for i in range(1, MAX_ITERATIONS):
    img = robot.grab_image()

    name = f"calibration_images/hp_img_{i}.png"
    cv2.imwrite(name, img)

    img = cv2.resize(img, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    cv2.imshow(name, img)
    cv2.waitKey(0)

