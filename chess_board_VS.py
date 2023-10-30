#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class ChessboardDetector:
    def __init__(self):
        self.bridge = CvBridge()

        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.detect_roi)

        self.prev_roi_area = 0  # initial previous ROI area
        self.last_printed_direction = ""  # store last moving direction
        self.last_depth = ""

        self.latest_depth_image = None

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
            return

    def combined_visual_servoing(self, x, y, w, h, img_width, img_height, depth_value):
        # Visual Servoing for Position
        center_x = x + w // 2
        center_y = y + h // 2
        img_center_x = img_width // 2
        img_center_y = img_height // 2
        dx = center_x - img_center_x
        dy = center_y - img_center_y

        direction = ""
        if abs(dx) > 20:  # Threshold
            direction += "Move camera to the left" if dx < 0 else "Move camera to the right"
        if abs(dy) > 20:  # Threshold
            direction += " & " if direction else ""
            direction += "Move camera up" if dy < 0 else "Move camera down"

        # Depth Visual Servoing
        desired_distance = 500  # 50cm
        tolerance = 50  # 5cm

        depth_direction = ""
        if depth_value > desired_distance + tolerance:
            depth_direction = "move closer"
        elif depth_value < desired_distance - tolerance:
            depth_direction = "move further"
        else:
            depth_direction = "in desired range"

        # Combine the two directions
        combined_direction = direction
        if depth_direction:
            combined_direction += " & " if combined_direction else ""
            combined_direction += depth_direction

        # Check if the chessboard is centered and in the desired range
        if not direction and depth_direction == "in desired range":
            combined_direction = "centered & in desired range"

        # Only print if the direction has changed from the last one or if it's "centered & in desired range"
        if combined_direction != self.last_printed_direction or combined_direction == "centered & in desired range":
            print(combined_direction)
            self.last_printed_direction = combined_direction

    # detect the chess board within ROI
    def detect_chess_board_within_roi(self, roi_image):
        # check if roi image is None
        if roi_image is None or roi_image.size == 0 or roi_image.shape[0] == 0 or roi_image.shape[1] == 0:
            rospy.loginfo("ROI image is empty or invalid. Skipping this frame.")
            return

        # narrow down the ROI area
        h, w = roi_image.shape[:2]
        roi_image = roi_image[70:h - 70, 70:w - 70]

        # detect the black square in the reduced roi image
        lower_black = np.array([0, 0, 0], dtype="uint8")
        upper_black = np.array([130, 130, 130], dtype="uint8")
        black_mask = cv2.inRange(roi_image, lower_black, upper_black)

        # get shape
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # if found enough black areas, try to get a rectangle that surrounds them
        if len(contours) > 0:
            x_min = y_min = float('inf')
            x_max = y_max = float('-inf')

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                x_min = min(x_min, x)
                y_min = min(y_min, y)
                x_max = max(x_max, x + w)
                y_max = max(y_max, y + h)

            # draw rectangle that surround all the black areas
            cv2.rectangle(roi_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)


    # detect ROI (region of interest)
    def detect_roi(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # set RGB
        lower_black = np.array([0, 0, 0], dtype="uint8")
        upper_black = np.array([100, 100, 100], dtype="uint8")

        # color filter
        black_mask = cv2.inRange(cv_image, lower_black, upper_black)

        # get contours
        contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # drawing bounding box
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 10000:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                if len(approx) == 4:  # find quad
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h
                    inverse_aspect_ratio = h / w  # considering the vertical orientation

                    # Check for both horizontal and vertical orientations
                    if (1.0 < aspect_ratio < 2.0) or (1.0 < inverse_aspect_ratio < 2.0):  # check if this is a rectangle
                        new_roi_area = w * h
                        if abs(new_roi_area - self.prev_roi_area) < self.prev_roi_area * 0.5:  # set -> 50%
                            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # draw bounding box

                            # Check if the bounding rectangle is within the image bounds
                            if x >= 0 and y >= 0 and x + w <= cv_image.shape[1] and y + h <= cv_image.shape[0]:
                                roi_image = cv_image[y:y + h, x:x + w]
                                if roi_image.size != 0:  # Check if the ROI is not empty
                                    self.detect_chess_board_within_roi(roi_image)

                                    # get central depth
                                    center_x = x + w // 2
                                    center_y = y + h // 2

                                    # Modify the depth value extraction
                                    if self.latest_depth_image is not None:
                                        depth_value = self.latest_depth_image[center_y, center_x]

                                        # Call the visual servoing function
                                        self.combined_visual_servoing(x, y, w, h, cv_image.shape[1], cv_image.shape[0],
                                                                      depth_value)

                                else:
                                    rospy.loginfo("Skipped an empty ROI.")
                            else:
                                rospy.loginfo("ROI coordinates are out of bounds.")

                        self.prev_roi_area = new_roi_area  # update previous ROI's area

        # show img
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)


def main():
    rospy.init_node('chessboard_detector', anonymous=True)
    cd = ChessboardDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
