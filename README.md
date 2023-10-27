# 41014_Sensors-Control

## RGB-D Camera - Visual servoing

### Environment and device
Environment: Ubuntu 20.04 <br> Platform: ROS noetic (Python3) <br> Language: Python3 <br> Device: Intel Realsense D435 (RGB-D camera)

### Setup Instructions

To set up the environment for running the ROS package, follow these detailed steps:

1. **Downloading and Preparation:**
   - Start by downloading the ROS package folder named `camera_d435_haiyang`.
   - This folder contains all the necessary files and scripts required for the project.

2. **Workspace and Package Setup:**
   - Navigate to your personal ROS workspace, which is typically named `catkin_ws`. If you haven't already created `catkin_ws`, you should do so through the ROS installation and environment setup.
   - Inside `catkin_ws`, locate the `src` directory. This is where all your ROS packages are stored.
   - Move or copy the `camera_d435_haiyang` folder into the `src` directory of your `catkin_ws`.

3. **Project Files Arrangement:**
   - Within the `camera_d435_haiyang` package, find the `src` folder.
   - Now, navigate to the "RGB_D_VisualServing" project within the `src` directory of `camera_d435_haiyang`.
   - Transfer all the `.py` script files you've downloaded into this "RGB_D_VisualServing" folder.

4. **ROS Package Renaming and Configuration (Optional):**
   - You have the option to rename the `camera_d435_haiyang` package to a name of your preference. However, if you decide to do so, there are a couple of important files you must update accordingly:
     - `package.xml`: This file contains meta-information about the package. If you change the package's name, you must update the name tag in this file with the new package name.
     - `CMakeLists.txt`: This file contains instructions for building your package. Ensure that it is up to date with any changes you've made, particularly if libraries or dependencies have been modified or added.

5. **Building the Package:**
   - Once you have all the files in the proper directories, open a terminal.
   - Navigate to your `catkin_ws` directory using the command `cd ~/catkin_ws`.
   - Now, use the command `catkin_make` to build your workspace. This step is essential for ROS to recognize and manage the new package.

6. **Post-Build Steps:**
   - After successfully building the workspace, don't forget to source your workspace's setup script using the command `source devel/setup.bash`. This step makes the new package available to your ROS environment.

7. **Verification:**
   - To verify that everything is set up correctly, you can use the command `rospack list` to see if your new package is listed among the available ROS packages.

**Note:** These instructions assume that you have a functioning ROS environment and the necessary permissions and dependencies installed. If you encounter errors related to missing dependencies, you may need to install them using `rosdep install --from-paths src --ignore-src -r -y` inside your `catkin_ws` directory.

### How to launch
1. `roslaunch realsense2_camera rs_camera.launch`
2. `rosrun camera_d435_haiyang chess_board_VS.py`

### Individual contribution to the code
Haiyang:
1. Git repo setup & generated `README.md` file
2. Coke can detection and visual seroving
3. Chess board detection
   
Anh:
1. Chess board visual seroving coding

Harley:
1. Chess board visual seroving testing/debugging
