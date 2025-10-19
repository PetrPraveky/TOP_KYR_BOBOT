import numpy as np
from ctu_crs import CRS97
import cv2
from perception import *
from aruco_detection import *
	
STARTING_POSITION = np.array([0,	-26.2431000,	-109.422900,
							  0,	-44.6022000, 	0])

def move_relative_distances(robot, move_vector):
	current_pose = robot.fk(robot.get_q())

	current_pose[:3,3] += np.array(move_vector)
	ik_sols = robot.ik(current_pose)

	if len(ik_sols) < 1:
		print("Machine impossible")
		return 1

	q0 = robot.q_home
	closest_solution = min(ik_sols, key=lambda q: np.linalg.norm(q - q0))

	robot.move_to_q(closest_solution)
	return 0

def move_to_start(robot):
	robot.move_to_q(np.deg2rad(STARTING_POSITION))


# Initialize the robot interface
robot = CRS97()

# test = input("zapni ruku")
robot.initialize(home=False)

img = robot.grab_image()

if img is not None:
	print(f"Successfully grabbed image with shape: {img.shape}")

# Useful position
#robot.move_to_q(np.deg2rad(np.array([0,-20,-100,0,-60,0])))
#robot.move_to_q(np.deg2rad(np.array([-30,-30, -30, -30,-30,-30])))
# move_relative_distances(robot, [0, 0, -0.1])

# print(robot.fk(robot.get_q()))
# print(np.rad2deg(robot.get_q()))

# move_to_start(robot)
move_vector = np.array([0, 0, 0.2])

move_relative_distances(robot, move_vector)

robot.wait_for_motion_stop()
# move_to_start(robot)


img = cv2.resize(img, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

# cv2.imwrite("obrazek.png", img)
# circle_centers, radiuses = find_hoop(img)
# aruco_detection(img)

cv2.imshow("Muj oblibeny obrazek", img)
cv2.waitKey(0)

robot.close()
