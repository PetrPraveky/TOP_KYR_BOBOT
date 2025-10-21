import cv2
import numpy as np
import os

def get_K_matrix(images):
    dict_ = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board = cv2.aruco.CharucoBoard((5,5), 0.025, 0.018, dict_)

    detector_params = cv2.aruco.DetectorParameters()
    charuco_params = cv2.aruco.CharucoParameters()
    detector = cv2.aruco.CharucoDetector(board, charucoParams=charuco_params, detectorParams=detector_params)

    obj_points = []
    img_points = []
    imsize = None

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if imsize is None:
            imsize = (gray.shape[1], gray.shape[0])

        ch_corners = np.array([])
        ch_ids = np.array([])

        ch_corners, ch_ids, mk_corners, mk_ids = detector.detectBoard(gray)
        print(ch_corners, ch_ids)
        if ch_corners is None or ch_ids is None or len(ch_ids) < 1:
            print("Not enough ChAruCo corners!")
            continue

        cv2.aruco.drawDetectedCornersCharuco(gray, ch_corners, ch_ids)
        cv2.imshow("charuco", gray)
        cv2.waitKey(0)

        # 3D rohy šachovnice (Z=0) – vyber podle indexů ch_ids
        all_obj = getattr(board, "chessboardCorners", board.getChessboardCorners())
        obj = all_obj[ch_ids.flatten()].astype(np.float32)      # (N,3)
        imgp = ch_corners.reshape(-1,2).astype(np.float32)      # (N,2)

        obj_points.append(obj)
        img_points.append(imgp)

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, imsize, None, None)
    print("reproj err:", ret, "\nK=\n", K, "\ndist=", dist.ravel())


if __name__ == '__main__':
    images = []
    folder = "./../calibration_images"
    for name in sorted(os.listdir(folder)):
        if name.lower().endswith(".png"):
            path = os.path.join(folder, name)
            img = cv2.imread(path) 
            if img is not None:
                images.append(img)

    get_K_matrix(images)