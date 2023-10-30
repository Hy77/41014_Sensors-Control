Author: Haiyang SUN

**Short Description of the Code:**

The code is structured as a ROS node written in Python, leveraging the OpenCV library for image processing. The primary class, `ChessboardDetector`, initializes ROS subscribers for RGB and depth images from a camera. Within this class:

- `depth_callback`: Acquires the depth image and converts it to a usable format.
- `detect_chess_board_within_roi`: Narrows down the search within the identified region of interest (ROI) to detect the chessboard pattern.
- `combined_visual_servoing`: Provides feedback on camera adjustments based on the position and depth of the detected chessboard.
- `detect_roi`: Identifies potential ROIs in the RGB image that might contain the chessboard and then calls the function to detect the actual chessboard within these ROIs.

The `main` function initializes the ROS node and the `ChessboardDetector` class, and then keeps the node running.

---

**README.md:**

# ROS Chessboard Detector with OpenCV

## Overview
A ROS node designed to detect a chessboard in a video stream using OpenCV. It identifies potential regions in the RGB image that might contain a chessboard, detects the actual chessboard within these regions, and provides feedback for camera adjustments based on the chessboard's position and depth.

## Structure:
- **ChessboardDetector Class**: Main class handling the detection process.
  - `depth_callback`: Handles depth image data.
  - `detect_chess_board_within_roi`: Detects the chessboard within a given ROI.
  - `combined_visual_servoing`: Provides camera adjustment feedback.
  - `detect_roi`: Identifies potential ROIs and detects the chessboard.

## Prerequisites:
- Python 3
- ROS (Robot Operating System) Noetic
- OpenCV
- cv_bridge package for ROS
