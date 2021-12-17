#!/usr/bin/env python

import cv2
import numpy as np
from geometry_msgs.msg import Point

def blob_search(image_raw, color):

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    
    if(color == 'burger'):
        lower = (100, 50, 80)  
        upper = (170, 200, 220) 
    if(color == 'bun'):
        lower = (50, 50, 80)  
        upper = (150, 200, 270) 

    mask_image =  cv2.inRange(hsv_image, lower, upper)

    # Edit Below
    ##################################
    # Setup SimpleBlobDetector parameters by editing the following code:
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = False

    # Filter by Circularity
    params.filterByCircularity = True

    # Filter by Inertia
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # Edit Above
    ##################################

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect keypoints
    keypoints = detector.detect(mask_image)
    i = len(keypoints)
    if i == 0:
        r = None
        c = None
    elif i == 1:
        keypoint = keypoints[0]
        c = keypoint.pt[0]
        r = keypoint.pt[1]
    else:
        keypoint = keypoints[0]

        # Get x and y
        c = keypoint.pt[0]
        r = keypoint.pt[1]

    im_with_keypoints = image_raw

    if len(keypoints) == 0:
        im_with_keypoints = image_raw
    else:
        # Feel free to use these as the color that you draw your keypoints and circle
        if color == 'yellow':
            draw_color = (255, 0, 0)
        else:
            draw_color = (255, 0, 255)
    
    # Show masked image
    im_mask = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR)
    cv2.namedWindow("Masked Image")
    cv2.imshow("Masked Image", im_mask)

    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, color = (0, 255, 0), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Note to students for pressing enter to continue
    im_with_keypoints = cv2.putText(im_with_keypoints, 'Press Enter to Continue', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.namedWindow("Press Enter to Continue")
    cv2.imshow("Press Enter to Continue", im_with_keypoints)

    while True:
        key = cv2.waitKey(0)
        if key == 13:
            cv2.destroyAllWindows()
            break

    return r,c
