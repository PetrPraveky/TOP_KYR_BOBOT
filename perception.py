from typing import List
from numpy.typing import ArrayLike
import numpy as np
import cv2 


def find_hoop(img: ArrayLike) -> np.ndarray:
    """
    Find homography based on images containing the hoop and the hoop positions loaded from
    the hoop_positions.json file in the following format:
    """

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
        for (x, y, r) in circles[0, :]:
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            cv2.circle(img, (x, y), 2, (0, 0, 255), 3)

        x, y, r = circles[0][0]
        return ((x,y), r)
    else:
        print("Hoop not found")
        return (None, None)

def find_homography():
    # end_points = np.array([[pose['translation_vector'][0],pose['translation_vector'][1]] for pose in hoop_positions])

    # H, mask = cv2.findHomography(circle_center, end_points, method=0, ransacReprojThreshold=5.0, maxIters=2000, confidence=0.995)
    # homographies.append(H)


    return None


