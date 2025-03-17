# Camera 2D LiDAR Calibration Library

This is a Camera and 2D LiDAR calibration library that computes the SE(3) transformation from the camera frame to the 2D LiDAR frame. 

## Assumptions

This calibration method relies on the following assumptions:

1. A square chessboard pattern, with known real-world square size and number of squares, is within full-view of the camera. A copy of this chessboard is provided in `chessboard.pdf`.
2. The LiDAR is scanning the world in a plane that is parallel to the ground.
3. The chessboard is placed perpendicular to the ground, and is oriented horizontally along its width, with its lowest portion parallel to the ground.
4. The chessboard is supported by a wall with straight edges that are in-view of the camera and discernible from the surroundings.
5. The camera is oriented such that the left/right sides of the wall appear in the left/right sides of the image respectively.
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
cd ~/ros2_ws
colcon build --packages-select camera_2d_lidar_calibration # build ROS dependencies
source install/setup.bash
```

## Usage

To use this package, place the desired bags in `~/ros2_ws/src/camera_2d_lidar_calibration/bags/`. Ensure that the `$HOME` path variable is set correctly. Then, modify the `camera_intrinsic_matrix` and `distortion_coefficients` in `~/ros2_ws/src/camera_2d_lidar_calibration/config/params.yaml` to match the camera's parameters. These coefficients can be obtained from any standard camera calibration method.

After setting the parameters, run the following:

```
ros2 launch camera_2d_lidar_calibration camera_2d_lidar_calibration.launch.py
```


After running, an interface will appear with instructions on selecting 2D LiDAR points that represent the wall containing the chessboard pattern. To select the points, first change the sliders that control the starting and ending indices of scans from the ROS bag to desired values. Then, use the Zoom feature to zoom into a particular region and click `Select Points`. Once finished, click `Done` and repeat this for the other ROS bags.


<video src="readme_pictures/recording.webm" height=200></video>

<p align="center">
<img src="readme_pictures/lidar_2d_selection_menu.png" height="400">
<img src="readme_pictures/lidar_2d_selection.png" height="400">
</p>

<p align="center">
<img src="readme_pictures/lidar_2d_selection_red.png" height="400">
<img src="readme_pictures/lidar_2d_selection_zoom_out.png" height="400">
</p>

To correspond the 2D LiDAR points with the camera frame, an interface will similarly appear with instructions on selecting lines that correspond to the edges of the wall. This interface works similarly to the previous interface.

<p align="center">
<img src="readme_pictures/wall_edge_menu.png" height="400">
<img src="readme_pictures/wall_edge_selection.png" height="400">
</p>

After following these instructions, plots will appear for each bag that represent the fitted lines corresponding to the wall.

<p align="center">
<img src="readme_pictures/lidar_detected_wall.png" height="400">
<img src="readme_pictures/camera_detected_wall.png" height="400">
</p>

As a sanity check, a set of test points from the camera frame are projected into the LiDAR frame.

<p align="center">
<img src="readme_pictures/test_points.png" height="400">
<img src="readme_pictures/transformed_points.png" height="400">
</p>

The resulting calibration is saved in `~/ros2_ws/src/camera_2d_lidar_calibration/results/camera_2d_lidar_calibration_transformation_result_{timestamp}/`, alongside any relevant data. 

Evidently, the transformed points are close albeit can differ from any individual ROS bag. Recording more ROS bags and selecting the LiDAR points more carefully will improve the accuracy.

## How It Works

After selecting the 2D LiDAR points, the RANSAC algorithm from scikit-learn (https://scikit-learn.org/stable/auto_examples/linear_model/plot_ransac.html) is used to robustly find the wall line when outliers can exist. This can fail if the gradient is infinite, so the axis with smallest range is used as the dependent variable.

The vertical edges are found using a Probabilistic Hough Transform, followed by a filtering by the line's gradient to select the candidate vertical lines. After selecting the lines, the first principal component is computed using singular value decomposition, which computes the line that minimises the L2-norm to all lines' endpoints. This new line is used as a single representative for a vertical edge of the wall.

After computing the vertical edges, the centre line of the chessboard is projected in the camera frame until it intersects the plane spanned by the vertical edges and the common normal to the chessboard and vertical edge. This corresponds to an estimated correspondence with the edges of the wall that were detected by the 2D LiDAR scans.

After finding these correspondence across all ROS bags, the SE(3) transformation and scaling is computed using `cv2.estimateAffine3D(wall_edge_camera_frame_points, wall_edge_lidar_points, force_rotation=True)`.

