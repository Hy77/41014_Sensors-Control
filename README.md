# 41014_Sensors-Control

## RGB-D Camera - Visual servoing

### Environment and device
Environment: Ubuntu 20.04 <br> Platform: ROS noetic (Python3) <br> Language: Python3 <br> Device: Intel Realsense D435 (RGB-D camera)

### How to setup
Download the 'camera_d435_haiyang' folder(ros package), and put all '.py' files into the "'username'/catkin_ws/src/camera_d435_haiyang/src/RGB_D_VisualServing" folder.

'catkin_ws' is my own workspace <br>
'camera_d435_haiyang' is my ros package <br>
'RGB_D_VisualSeroving' is my Python project <br>

Note: you may change the ros package name to whatever you want but make sure you keep other files like package.xml & CMakeLists.txt are up to date.

### How to launch
1. roslaunch realsense2_camera rs_camera.launch
2. rosrun camera_d435_haiyang (filename.py) eg: chess_board_VS.py

