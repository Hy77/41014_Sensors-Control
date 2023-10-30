**Author**: Haiyang SUN

**Short Description of the Code**:

The code is structured as a ROS node written in Python, leveraging the OpenCV library for image processing. The primary purpose is to detect a coke can in a video stream. The main functions include:

- `visual_servoing`: Determines the direction to adjust the camera based on the coke can's position relative to the center of the image.
- `callback_color`: Processes the RGB image from the ROS topic, detects the coke can based on its color, and calls the `visual_servoing` function.
- `main`: Initializes the ROS node, subscribes to the RGB image topic, and keeps the node running.

---

**README.md**:

# ROS Coke Can Detector with OpenCV

## Overview
A ROS node designed to detect a coke can in a video stream using OpenCV. The script processes the RGB image from a ROS topic, identifies the coke can based on its color, and provides feedback for camera adjustments based on the can's position.

## Structure:
- **visual_servoing**: Determines camera adjustment direction.
  - **Parameters**:
    - `x, y`: Top-left coordinates of the bounding box.
    - `w, h`: Width and height of the bounding box.
    - `img_width, img_height`: Image dimensions.
  - **Output**: Camera adjustment direction.
  
- **callback_color**: Processes RGB image and detects coke can.
  - **Parameters**:
    - `msg`: Image message from ROS topic.
  - **Output**: Video stream with highlighted coke can.

- **main**: Initializes ROS node and manages subscriptions.
  - **Output**: Processes video stream continuously.

## Prerequisites:
- Python 3
- ROS (Robot Operating System) Noetic
- OpenCV
- cv_bridge package for ROS

---
