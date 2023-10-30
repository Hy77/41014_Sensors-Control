**Author**: Haiyang SUN

**Short Description of the Code**:

The code is structured as a ROS node written in Python, leveraging the OpenCV library for image processing. The primary purpose is to detect a coke can in a video stream based on its color and shape, especially under optimal lighting conditions. The main functions include:

- `coke_shape_detection`: Determines if the detected object is a coke can based on its shape and depth information.
- `lighting_true_detection`: Detects the coke can under optimal lighting conditions using its color and calls the `coke_shape_detection` function.
- `callback`: Processes both the RGB and depth images from the ROS topics and calls the `lighting_true_detection` function.
- `main`: Initializes the ROS node, subscribes to both RGB and depth image topics, and synchronizes them using `ApproximateTimeSynchronizer`.

---

**README.md**:

# ROS Coke Can Detector with OpenCV

## Overview
A ROS node designed to detect a coke can in a video stream using OpenCV. The script processes both RGB and depth images from ROS topics, identifies the coke can based on its color and shape, especially under optimal lighting conditions, and provides feedback on the detection results.

## Structure:
- **coke_shape_detection**: Determines if the detected object is a coke can.
  - **Parameters**:
    - `w1, h1`: Width and height of the bounding box.
    - `mean_depth`: Mean depth within the bounding box.
  - **Output**: Boolean indicating if the object is a coke can.

- **lighting_true_detection**: Detects coke can under optimal lighting.
  - **Parameters**:
    - `img_color`: RGB image.
    - `img_depth`: Depth image.
  - **Output**: Video stream with potential coke can highlighted.

- **callback**: Processes RGB and depth images.
  - **Parameters**:
    - `msg_color`: RGB image message from ROS topic.
    - `msg_depth`: Depth image message from ROS topic.
  - **Output**: Calls `lighting_true_detection` function.

- **main**: Initializes ROS node and manages subscriptions.
  - **Output**: Processes video stream continuously.

## Prerequisites:
- Python 3
- ROS (Robot Operating System) Noetic
- OpenCV
- cv_bridge package for ROS

---
