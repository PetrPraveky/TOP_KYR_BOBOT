import cv2
import numpy as np
import os

def aruco_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    corners, ids, rejected = detector.detectMarkers(gray)
    img_corners = img.copy()

    print("Corners:", corners)
    print("Ids: ", ids)

    if (len(corners) != 2):
        print("EH nefunguje")
        return

    # Get target direction
    # Detect centers
    aruco_centers = []
    for marker in corners:
        aruco_center = [0, 0]
        for corner in marker[0]:
            aruco_center += corner

        aruco_center /= 4
        aruco_centers.append(aruco_center)

    print("Centers:", aruco_centers)

    target_center = np.array([aruco_centers[0] + aruco_centers[1]]) / 2

    print("Target center:", target_center)

    id1 = np.array(ids).tolist().index(min(ids))
    id2 = np.array(ids).tolist().index(max(ids))
    print("L:", id1, "H:", id2)
    
    v = (aruco_centers[id2] - aruco_centers[id1])
    n = np.linalg.norm(v)
    u = v / n # Normalisation
    
    # Vector rotation
    c = np.cos(np.deg2rad(45))
    s = np.sin(np.deg2rad(45))

    rot = np.array([c*u[0] + s*u[1], -s*u[0] + c*u[1]])

    # Angle
    angle = np.rad2deg(np.arctan(rot[1]/rot[0]))
    print("Angle:", (angle - 90) % 360)


    p0 = tuple(np.round(target_center).astype(int)[0])
    p1 = tuple(np.round(target_center + rot * 100).astype(int)[0])


    print(p0, p1)

    cv2.aruco.drawDetectedMarkers(img_corners, corners, ids)
    cv2.arrowedLine(img_corners, p0, p1, (0,0,255), 4, tipLength=0.18)

    img_corners = cv2.resize(img_corners, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    cv2.imshow("aruco", img_corners)
    cv2.waitKey(0)






if __name__ == '__main__':
    folder = "./../targets"
    for name in sorted(os.listdir(folder)):
        if name.lower().endswith(".png"):
            path = os.path.join(folder, name)
            img = cv2.imread(path) 
            if img is not None:
                # images.append(img)
                aruco_detection(img)