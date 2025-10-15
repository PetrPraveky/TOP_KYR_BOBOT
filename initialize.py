import numpy as np
from ctu_crs import CRS97
import cv2

# Initialize the robot interface
robot = CRS97()
robot.initialize()

img = robot.grab_image()

if img is not None:
	print(f"Successfully grabbed image with shape: {img.shape}")

robot.move_to_q(np.deg2rad(np.array([0,0,-90,0,-90,0])))

half_img = cv2.resize(img, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

cv2.imshow("Muj oblibeny obrazek", half_img)
cv2.waitKey(0)

robot.close()