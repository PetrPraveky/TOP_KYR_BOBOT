#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2025-09-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
from typing import List
from numpy.typing import ArrayLike
import numpy as np
import cv2  # noqa
from copy import deepcopy


def find_hoop_homography(images: ArrayLike, hoop_positions: List[dict]) -> np.ndarray:
    """
    Find homography based on images containing the hoop and the hoop positions loaded from
    the hoop_positions.json file in the following format:

    [{
        "RPY": [-0.0005572332585040621, -3.141058227474627, 0.0005185830258253442],
        "translation_vector": [0.5093259019899434, -0.17564068853313258, 0.04918733225140541]
    },
    {
        "RPY": [-0.0005572332585040621, -3.141058227474627, 0.0005185830258253442],
        "translation_vector": [0.5093569397977782, -0.08814069881074972, 0.04918733225140541]
    },
    ...
    ]
    """

    images = np.asarray(images)
    assert images.shape[0] == len(hoop_positions)    
    # todo HW03: Detect circle in each image
    # todo HW03: Find homography using cv2.findHomography. Use the hoop positions and circle centers.

    circle_centers = []
    # radiuses = []
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Blur to reduce noise - very good feature
        gray = cv2.medianBlur(gray, 9)

        # Detect circles
        circles = cv2.HoughCircles(gray,
                                cv2.HOUGH_GRADIENT,
                                dp=1,             # Inverse accumulator resolution
                                minDist=100,       # Minimum distance between circles
                                param1=50,        # Canny high threshold
                                param2=50,        # Accumulator threshold
                                minRadius=80,     # Minimum radius
                                maxRadius=200)    # Maximum radius

        # Draw results
        if circles is not None:
            # circles = np.uint16(np.around(circles))
            # for (x, y, r) in circles[0, :]:
            #     cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            #     cv2.circle(img, (x, y), 2, (0, 0, 255), 3)
            x, y, r = circles[0][0]
            circle_centers.append((x,y))
            # radiuses.append(r)

    circle_centers = np.array(circle_centers)
    end_points = np.array([[pose['translation_vector'][0],pose['translation_vector'][1]] for pose in hoop_positions])

    H, mask = cv2.findHomography(circle_centers, end_points, method=0, ransacReprojThreshold=5.0, maxIters=2000, confidence=0.995)
    # homographies.append(H)
    
    


    return H
