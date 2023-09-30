#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ChessboardDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # set RGB
        lower_black = np.array([0, 0, 0], dtype="uint8")
        upper_black = np.array([50, 50, 50], dtype="uint8")

        # color filter
        black_mask = cv2.inRange(cv_image, lower_black, upper_black)

        # get contours
        contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # drawing bounding box
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 5000:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                if len(approx) == 4:  # find quad
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h
                    if 0.5 < aspect_ratio < 1.5:  # check if this is a rec
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 画出bounding box

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
