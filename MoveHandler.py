import numpy as np
import cv2
from perception import *
from aruco_detection import *


class MoveHandler:
  def __init__(self, robot):
    self.robot = robot
    self.CALIBRATION_POSITION = np.array([-1.800000e-03, -5.780970e+01, -1.121427e+02,  0.000000e+00, -1.028700e+01,
  0.000000e+00])
    self.STARTING_POSITION = np.array([4.50000000e-03, -2.48202000e+01, -1.09159200e+02,  1.78217822e-03,
 -4.62528000e+01, 3.56435644e-03])
    self.NEW_HOME_POSITION = np.array([-39.3777,     -40.6629,     -83.1753,      -0.18178218, -56.3454,
 -39.27029703])
    self.FIVE_POSITION = np.array([-4.50000000e-03,  1.71000000e-02, -8.99973000e+01,  9.00017822e+01,
 -9.49968000e+01,  0.00000000e+00])

  def move_to_q_position(self, q_position=None,five_pos=False):
    if q_position is None:
      q_position = self.STARTING_POSITION
    
    if five_pos == True:
      q_position = self.FIVE_POSITION
    
    self.robot.move_to_q(np.deg2rad(q_position))
    return 0
  
  def move_home(self):
    self.robot.move_to_q(self.robot.q_home)
    return 0

  def move_new_home(self):
    self.robot.move_to_q(self.NEW_HOME_POSITION)
    return 0
  
  def move_to_relative_position(self, move_vector):
    current_pose = self.robot.fk(self.robot.get_q())

    current_pose[:3,3] += np.array(move_vector)
    ik_sols = self.robot.ik(current_pose)

    if len(ik_sols) < 1:
      print(f"{__file__} Machine impossible")
      return 1

    q0 = self.robot.q_home
    closest_solution = min(ik_sols, key=lambda q: np.linalg.norm(q - q0))

    self.robot.move_to_q(closest_solution)
    return 0
  
  def move_ik(self, position, matrix=None):
    pose = self.robot.fk(self.robot.get_q())
    pose[:3,3] = position
  
    if matrix is not None:
      pose[:3,:3] = matrix

    ik_sols = self.robot.ik(pose)
    new_ik_sols = []
    for sol in ik_sols:
      if 155 < abs(np.rad2deg(sum(sol[:-1]))) % 360 < 205 and self.robot.in_limits(sol):
        new_ik_sols.append(sol)

    ik_sols = new_ik_sols

    if len(ik_sols) < 1:
      print(f"{__file__} Machine impossible")
      return 1

    q0 = self.robot.q_home
    closest_solution = min(ik_sols, key=lambda q: np.linalg.norm(q - q0))

    self.robot.move_to_q(closest_solution)

    return 0
    
  def move_relative_sequantial(self, instructions):
    for instruction in instructions:
      self.move_to_relative_position(instruction)
      self.robot.wait_for_motion_stop()

    return 0

  def rotate_ik(self, rotation_matrix):
    pose = self.robot.fk(self.robot.get_q())
    print(pose)
    pose[:3,:3] = rotation_matrix @ pose[:3,:3]
    print(pose)
    ik_sols = self.robot.ik(pose)

    if len(ik_sols) < 1:
      print(f"{__file__} Machine impossible")
      return 1

    q0 = self.robot.q_home
    closest_solution = min(ik_sols, key=lambda q: np.linalg.norm(q - q0))

    self.robot.move_to_q(closest_solution)
    return 0
  
  def check_instructions(self, move_vectors):
    pose = self.robot.fk(self.robot.get_q())
    for vector in move_vectors:
      pose[:3,3] += np.array(vector)
      sol = self.robot.ik(pose)
      self.robot.in_limits(sol)