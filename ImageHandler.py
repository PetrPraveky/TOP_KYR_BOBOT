import numpy as np
import cv2
from perception import *
from aruco_detection import *
from MoveHandler import MoveHandler
class ImageHandler:
  def __init__(self, robot):
    self.robot = robot
    self.last_img = None
    self.imgs = []
    self.load_h()
    self.homography = np.array(self.homography)
    self.target_pixels = None # (x,y) of the target

    self.aruco_centers = []
    self.aruco_corners = []
    self.aruco_ids = []
    self.aruco_ids_all = []

  def load_h(self, filename="homography.txt"):
    with open(filename) as f:
      self.homography = [list(map(float, line.split())) for line in f]
    return 0

  def save_h(self, filename="homography.txt"):
    with open(filename, "w") as f:
        for row in self.homography:
            f.write(" ".join(map(str, row)) + "\n")
    return 0

  def take_img(self):
    self.last_img = self.robot.grab_image()
    return self.last_img
  
  def take_imgs(self, imgs_cnt=10):
    self.imgs = []

    for _ in range(imgs_cnt):
      img = self.robot.grab_image()
      self.imgs.append(img)
      self.last_img = img
      self.show_img()
      input("For next image press enter")
    



  def save_img(self, img=None, filename="my_image.png"):
    if img is None:
      img = self.last_img
    cv2.imwrite(filename, img)

  def show_img(self, img=None, name="My image"):
    if img is None and self.last_img is not None:
      img = self.last_img
    else:
      print("No image to show")
      return 1

    img = cv2.resize(img, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



  def find_hoop(self,img, draw_results=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Blur to reduce noise - very good feature
    gray = cv2.medianBlur(gray, 5)

    # Detect circles
    circles = cv2.HoughCircles(gray,
                            cv2.HOUGH_GRADIENT,
                            dp=1,             # Inverse accumulator resolution
                            minDist=1000,       # Minimum distance between circles
                            param1=175,        # Canny high threshold
                            param2=50,        # Accumulator threshold
                            minRadius=80,     # Minimum radius
                            maxRadius=1000)    # Maximum radius

    if circles is not None:
        circles = np.uint16(np.around(circles))

        # Draw results
        if draw_results == True:
          for (x, y, r) in circles[0, :]:
              cv2.circle(img, (x, y), r, (0, 255, 0), 2)
              cv2.circle(img, (x, y), 2, (0, 0, 255), 3)

        x, y, r = circles[0][0]
        return ((x,y), r)
    else:
        print("Hoop not found")
        return (None, None)
    
  def find_homography(self, absolute_vectors) -> np.ndarray:
    images = np.asarray(self.imgs)

    circle_centers = []
    radiuses = []

    for img in images:
      center, radius = self.find_hoop(img)
      if center is None:
        continue

      circle_centers.append(center)
      radiuses.append(radius)

    circle_centers = np.array(circle_centers)
    end_points = np.array([[pose[0],pose[1]] for pose in absolute_vectors])

    H, mask = cv2.findHomography(circle_centers, end_points, method=0, ransacReprojThreshold=5.0, maxIters=2000, confidence=0.995)   
    self.homography = H

    return 0



  def find_target(self, img=None, draw_results=True):
    if img == None:
      img = self.last_img
  
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    corners, ids, rejected = detector.detectMarkers(gray)

    self.aruco_corners = corners
    self.aruco_ids = ids

    if draw_results:
      cv2.aruco.drawDetectedMarkers(img, corners, ids)

    aruco_centers = []

    if len(corners) == 0:
      print("Number of detected arucos is:", len(corners), "which is not 2!!!!")
      return 1
    elif (len(corners) == 1):
        print("Only one acuro!")
        # Create the one and only center
        aruco_center = [0, 0]
        for corner in corners[0][0]:
            aruco_center += corner
        aruco_center /= 4

        # Fill the id
        missingId = 1 if ids[0][0] == 2 else 2
        self.aruco_ids_all = np.array([[ids[0][0]], [missingId]])

        # Find vector to center
        vec = corners[0][0][0] - corners[0][0][2] if ids[0][0] == 2 else corners[0][0][2] - corners[0][0][0]
        mag = np.linalg.norm(vec)
        vec = vec / mag

        # Pixel length center to center
        length = (corners[0][0][0] - corners[0][0][1])
        lengthMag = np.linalg.norm(length)
        centerToCenter = lengthMag / 0.04 * np.sqrt(0.07 ** 2 + 0.07 ** 2)

        vec *= centerToCenter
        aruco_missing_center = aruco_center + vec

        aruco_centers.append(aruco_center)
        aruco_centers.append(aruco_missing_center)
    else:
      for corner in corners:
        aruco_center = [0,0]
        for pos in corner[0]:
          aruco_center[0] += pos[0]
          aruco_center[1] += pos[1]
        aruco_centers.append([aruco_center[0] / 4, aruco_center[1]/ 4])

        self.aruco_ids_all = ids


    self.aruco_centers = np.array(aruco_centers)
    self.target_pixels = ((aruco_centers[0][0] + aruco_centers[1][0]) / 2, (aruco_centers[0][1] + aruco_centers[1][1]) / 2)


    return 0

  def pixels_to_position(self, pixels):
    print(self.homography)
    print(self.homography.shape)
    return cv2.perspectiveTransform(pixels,self.homography)

  def get_target_angle(self):
    id1 = np.array(self.aruco_ids_all).tolist().index(min(self.aruco_ids_all))
    id2 = np.array(self.aruco_ids_all).tolist().index(max(self.aruco_ids_all))

    v = (self.aruco_centers[id2] - self.aruco_centers[id1])
    n = np.linalg.norm(v)
    u = v / n # Normalisation

    # Vector rotation
    c = np.cos(np.deg2rad(45))
    s = np.sin(np.deg2rad(45))

    rot = np.array([c*u[0] + s*u[1], -s*u[0] + c*u[1]])

    # Correct angle orientation
    orientation = -1 if rot[0] > 0 else 1
    offset = 90 if rot[1] < 0 else 0

    # Angle
    angle = np.rad2deg(np.arctan(rot[0]/rot[1]))

    p0 = tuple(np.round(self.target_pixels).astype(int))
    p1 = tuple(np.round(self.target_pixels + rot * 100).astype(int))
    cv2.arrowedLine(self.last_img, p0, p1, (0,0,255), 4, tipLength=0.18)
    
    x, y = rot[0], rot[1]
    if y > 0:
      result = -angle
    elif x < 0:
      result = 180 - angle
    elif x > 0:
      result = -(180 + angle)
    else:
      resul = 180


    print(angle, angle * orientation, result, offset, orientation)

    return result