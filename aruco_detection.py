import cv2
import numpy as np

def aruco_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    corners, ids, rejected = detector.detectMarkers(gray)

    cv2.aruco.drawDetectedMarkers(img, corners, ids)


if __name__ == '__main__':
    img = cv2.imread("./obrazek.png")
    print(img)