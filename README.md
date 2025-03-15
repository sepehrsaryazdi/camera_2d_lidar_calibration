# Camera 2D LiDAR Calibration Library

This is a Camera and 2D LiDAR calibration library that computes the SE(3) transformation from the camera frame to the 2D LiDAR frame. 

## Assumptions

This calibration method relies on the following assumptions:

1. A full square chessboard pattern with known real-world square size and number of squares is within full-view of the camera.
2. The LiDAR is scanning the world in a plane that is parallel to the ground.
3. The chessboard is placed perpendicular to the ground, and is oriented horizontally along its width, with its lowest portion parallel to the ground.
4. The chessboard is supported by a wall with straight edges that are in-view of the camera and discernible from the surroundings.
5. The camera is oriented such that the left/right sides of the wall appears in the left/right sides of the image, respectively.
6. The LiDAR scans are oriented so that ordering of wall sides agree with the camera, i.e. left and right.

To see a typical example that satisfies the assumptions above, refer to the following image corresponding to Turtlebot3 Burger. Its 2D LiDAR is located at the top of the robot and scans the world in a plane parallel to the ground.

<p align="center">
<img src="readme_pictures/thumbnail_IMG_5137.jpg" width="400">
</p>

In this example, both the chessboard and the edges of the wall are within full-view.

<p align="center">
<img src="readme_pictures/camera_pov.png" height="200">
<img src="readme_pictures/lidar_scans.png" height="200">
</p>


## Installation

To install, run the following code:

```
cd ~/ros2_ws/src
git clone https://github.com/sepehrsaryazdi/camera_2d_lidar_calibration.git
cd camera_2d_lidar_calibration
pip install -e . # install python dependencies
colcon build --packages-select camera_2d_lidar_calibration # build ROS dependencies
source install/setup.bash
```

## Usage

To use this package, place the desired bags in `~/ros2_ws/src/camera_2d_lidar_calibration/bags/`. Ensure that the `$HOME` path variable is set correctly. Then, run the following:

```
ros2 launch camera_2d_lidar_calibration camera_2d_lidar_calibration.launch.py
```

After running, an interface will appear with instructions on selecting 2D LiDAR points that represent the wall containing the chessboard pattern. To select the points, first change the sliders that control the starting and ending indices of scans from the ROS bag to a desired amount. Then, use the Zoom feature to zoom into a particular region and click `Select Points`. Once finished, click `Done` and repeat this for the other ROS bags.

<p align="center">
<img src="readme_pictures/lidar_2d_selection_menu.png" height="200">
<img src="readme_pictures/lidar_2d_selection.png" height="200">
</p>


<p align="center">
<img src="readme_pictures/lidar_2d_selection_red.png" height="200">
<img src="readme_pictures/lidar_2d_selection_zoom_out.png" height="200">
</p>

To correspond the 2D LiDAR points with the camera frame, an interface will similarly appear with instructions on selecting lines that correspond to the edges of the wall. This interface works similarly to the previous interface.

<p align="center">
<img src="readme_pictures/wall_edge_menu.png" height="200">
<img src="readme_pictures/wall_edge_selection.png" height="200">
</p>


