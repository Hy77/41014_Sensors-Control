#!/usr/bin/env python3

# Author: Haiyang Sun - 100% Codes

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

cv2.startWindowThread()

# Global variable
coke_can_message_printed = False  # set -> false, used to notify user found a coke can or not

def coke_shape_detection(w1, h1, mean_depth):

    # Check if mean_depth is zero or very close to zero
    if abs(mean_depth) < 1e-6:
        return False  # This is not a coke can

    # Define camera's focal length
    focal_length = 607 # mm

    # Define the size of a coke can
    h_real = 129  # mm
    w_real = 66  # mm

    # Estimate the real-world size of the object using the formula
    estimated_h_real = (h1 / focal_length) * mean_depth
    estimated_w_real = (w1 / focal_length) * mean_depth

    # Define a tolerance for size comparison
    tolerance = 0.5  # 50% tolerance

    # Check if the estimated size is within the tolerance of the actual size
    if (1 - tolerance) * h_real < estimated_h_real < (1 + tolerance) * h_real and \
            (1 - tolerance) * w_real < estimated_w_real < (1 + tolerance) * w_real:
        print(estimated_h_real, '&&', estimated_w_real)
        return True  # This is a coke can

    return False  # This is not a coke can

# this will use color & coke can's shape to detect the coke can in the optimal lighting environment
def lighting_true_detection(img_color, img_depth):

    global coke_can_message_printed

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)

    # Define range of red color in HSV
    lower_red1 = np.array([0, 100, 50])  # Decreased Saturation and Value
    upper_red1 = np.array([5, 255, 255])  # Kept the same
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([175, 100, 50])  # Decreased Saturation and Value
    upper_red2 = np.array([180, 255, 255])  # Kept the same
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # get darker red
    lower_red3 = np.array([160, 50, 20])  # Decreased Saturation and Value
    upper_red3 = np.array([180, 255, 255])
    mask_red3 = cv2.inRange(hsv, lower_red3, upper_red3)

    # combine all mask
    mask_combined = cv2.bitwise_or(mask_red1, mask_red2)
    mask_combined = cv2.bitwise_or(mask_combined, mask_red3)

    # Find contours, we only need the coke's edge
    # contours, _ = cv2.findContours(mask_combined, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Set a flag to track if a coke can is detected
    coke_can_detected = False

    # get into all countours
    for i, contour1 in enumerate(contours):
        # Calculate the area of the contour
        area1 = cv2.contourArea(contour1)

        # Filter contours based on area
        if area1 > 3000:  # filter out small red objects
            # Get the bounding box
            x1, y1, w1, h1 = cv2.boundingRect(contour1)

            # Extract depth information from the bounding box region
            depth_region = img_depth[y1:y1 + h1, x1:x1 + w1]
            mean_depth = np.nanmean(depth_region)  # Calculate the mean depth within the bounding box

            if coke_shape_detection(w1, h1, mean_depth):
                # This is likely the coke can, proceed with "bouncing box" and visual_servoing
                cv2.rectangle(img_color, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
                coke_can_detected = True  # Set the flag to True since a coke can is detected

    # Check the flag after the loop ends
    if not coke_can_detected:
        if not coke_can_message_printed:  # if the msg has not been printed
            print("Can not find any coke can...")
            coke_can_message_printed = True  # set to true, avoid multi-print
    else:
        coke_can_message_printed = False  # if detected, switch it

def callback(msg_color, msg_depth):
    bridge = CvBridge()

    # get rgb & depth image
    img_color = bridge.imgmsg_to_cv2(msg_color, "bgr8")
    img_depth = bridge.imgmsg_to_cv2(msg_depth, "32FC1")  # 32FC1 for depth image

    # Detect coke can under optimal lighting conditions
    lighting_true_detection(img_color, img_depth)

    # Show the video streaming(OUTPUT)
    cv2.imshow('Filtered Object Detection -> Coke can', img_color)
    cv2.waitKey(1)

def main():
    rospy.init_node('coke_can_detection', anonymous=True)

    # Subscribe to both rgb and depth image topics
    rgb_topic = "/camera/color/image_raw"
    depth_topic = "/camera/depth/image_rect_raw"

    # Use ApproximateTimeSynchronizer to synchronize the rgb and depth images
    from message_filters import ApproximateTimeSynchronizer, Subscriber

    sub_color = Subscriber(rgb_topic, Image)
    sub_depth = Subscriber(depth_topic, Image)

    ats = ApproximateTimeSynchronizer([sub_color, sub_depth], queue_size=5, slop=0.1)
    ats.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
