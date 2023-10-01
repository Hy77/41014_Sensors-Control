#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ChessboardDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.detect_roi)
        self.prev_roi_area = 0  # initial previous ROI area

    # detect the chess board within ROI
    def detect_chess_board_within_roi(self, roi_image):
        # narrow down the ROI area
        h, w = roi_image.shape[:2]
        roi_image = roi_image[70:h-70, 70:w-70]

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
        upper_black = np.array([130, 130, 130], dtype="uint8")

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
                    if 1.0 < aspect_ratio < 2.0:  # check if this is a rec
                        new_roi_area = w * h
                        if abs(new_roi_area - self.prev_roi_area) < self.prev_roi_area * 0.5:  # set -> 50%
                            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # draw bounding box
                            # get ROI img from the screen (coordinates)
                            roi_image = cv_image[y:y + h, x:x + w]
                            # now detect the chess board within ROI
                            self.detect_chess_board_within_roi(roi_image)
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
