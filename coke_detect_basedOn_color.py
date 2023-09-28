#Author: Haiyang SUN
#Date: 15 Sep 2023

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

cv2.startWindowThread()

last_direction = ""  # store last moving direction

def visual_servoing(x, y, w, h, img_width, img_height):
    global last_direction

    # Calculate the center of the bounding box
    center_x = x + w // 2
    center_y = y + h // 2

    # Calculate the center of the image
    img_center_x = img_width // 2
    img_center_y = img_height // 2

    # Calculate the distance between the center of the bounding box and the center of the image
    dx = center_x - img_center_x
    dy = center_y - img_center_y

    # Determine the direction to move the camera -> this also depends on the last direction
    direction = ""

    if abs(dx) > 20:  # Threshold
        direction += "Move camera to the left" if dx < 0 else "Move camera to the right"
    if abs(dy) > 20:  # Threshold
        direction += " and " if direction else ""
        direction += "Move camera up" if dy < 0 else "Move camera down"

    if direction:
        if direction != last_direction:  # Print only when the direction changes
            print(f"Direction to move: {direction}")
        last_direction = direction  # update last_direction
    else:
        print("Coke is centered")
        last_direction = ""  # init last_direction


def callback_color(msg):
    bridge = CvBridge()
    img_color = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)

    # Define range of red color in HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([5, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([175, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine the masks
    mask_combined = cv2.bitwise_or(mask_red1, mask_red2)

    # Find contours, we only need the coke's edge
    # contours, _ = cv2.findContours(mask_combined, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area, if its too small, then ignore it
    for i, contour1 in enumerate(contours):
        # Calculate the area of the contour
        area1 = cv2.contourArea(contour1)

        # Filter contours based on area
        if area1 > 1000:  # Adjust this value based on your specific requirements
            # Get the bounding box
            x1, y1, w1, h1 = cv2.boundingRect(contour1)

            # Draw the bounding box
            cv2.rectangle(img_color, (x1, y1), (x1+w1, y1+h1), (0, 255, 0), 2)

            # Call the visual_servoing function
            visual_servoing(x1, y1, w1, h1, img_color.shape[1], img_color.shape[0])

    # Show the video streaming(OUTPUT)
    cv2.imshow('Filtered Object Detection -> Coke can', img_color)
    cv2.waitKey(1)

def main():
    rospy.init_node('light_obj_detect', anonymous=True)

    # subscribe rgb image
    rgb_topic = "/camera/color/image_raw"
    rospy.Subscriber(rgb_topic, Image, callback_color)

    rospy.spin()

if __name__ == '__main__':
    main()
