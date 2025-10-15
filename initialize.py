import numpy as np
from ctu_crs import CRS97
import cv2
from perception import *
from aruco_detection import *

# Initialize the robot interface
robot = CRS97()

test = input("zapni ruku")
#robot.initialize()

img = robot.grab_image()

if img is not None:
	print(f"Successfully grabbed image with shape: {img.shape}")

# Useful position
#robot.move_to_q(np.deg2rad(np.array([0,-20,-100,0,-60,0])))
#robot.move_to_q(np.deg2rad(np.array([-30,-30, -30, -30,-30,-30])))
robot.wait_for_motion_stop()

img = cv2.resize(img, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

cv2.imwrite("obrazek.png", img)
circle_centers, radiuses = find_hoop(img)
aruco_detection(img)

cv2.imshow("Muj oblibeny obrazek", img)
cv2.waitKey(0)

robot.close()
