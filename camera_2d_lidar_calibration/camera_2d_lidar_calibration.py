import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rosbag2_py
from sensor_msgs.msg import Image, LaserScan
import os
from rclpy.serialization import deserialize_message
import cv2
import cv_bridge
import tkinter as tk
from tkinter import ttk
from pathlib import Path
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import matplotlib.pyplot as plt
import matplotlib.backends.backend_tkagg as tkagg 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.spatial.distance import pdist, squareform
import time
from mpl_toolkits.mplot3d import Axes3D
import laser_geometry
home = Path.home()





class ImageAndScans:
    def __init__(self, image:np.ndarray, scans:list[LaserScan]):
        assert isinstance(image, np.ndarray), "Error: image must be of the form cv2.self.root"
        assert isinstance(scans , list), "Error: scans must be a list."
        for scan in scans:
            assert isinstance(scan, LaserScan), "Error: scans must contain LaserScan objects."
        self.image_ = image
        self.scans_ = scans
        self.selected_points_bool_ = False
        self.selected_lidar_pc2_points_ = None

    def concatenate_scans_to_points(self, indices=[0]) -> np.ndarray:
        scans = self.get_scans()
        laser_projection = laser_geometry.LaserProjection()
        projected_scans = [laser_projection.projectLaser(scan) for scan in scans]
        scans_numpy = np.array([pc2.read_points(scan) for scan in projected_scans])
        scans_numpy = scans_numpy[np.array(indices)]
        concatenated_scans = np.concatenate(scans_numpy)
        return concatenated_scans # x,y,z,intensity,index
    
    def set_selected_lidar_points(self, selected_points:np.ndarray):
        self.selected_lidar_pc2_points_ = selected_points.copy()
        self.selected_points_bool_ = True

    def get_image(self) -> np.ndarray:
        return self.image_.copy()
    def get_scans(self) -> list[LaserScan]:
        return self.scans_.copy()
    
    def has_selected_points(self):
        return self.selected_points_bool_

    def get_selected_lidar_points(self):
        if self.selected_points_bool_:
            return self.selected_lidar_pc2_points_.copy()
        else:
            return None
    def copy(self):
        return ImageAndScans(self.get_image(), self.get_scans())
    


class SelectPointsInterface:
    """
    Class for selecting lidar points corresponding to back wall.
    """

    def __init__(self, image_and_scans:ImageAndScans):
        assert isinstance(image_and_scans, ImageAndScans), "Error: image_and_scans must be of type ImageAndScans."
        
        self.root = tk.Tk()
        self.root.title("Camera 2D LiDAR Calibration Menu")
        self.root.geometry("600x700")
        self.menubar = tk.Menu(self.root)        
        self.app = tk.Frame(self.root)

        self.image_and_scans = image_and_scans.copy()
        
        minimise_window_text = tk.Label(self.root,
                                            text="Use the slides to choose the starting and ending indices of 2D LiDAR scans respectively. \nSelect 2D LiDAR points corresponding to the perpendicular surface \nby using the Zoom feature followed by clicking 'Select Points'.\nOnce finished, click 'Done'.")
        minimise_window_text.pack(padx=5, pady=5)
        
        self.scan_start_slider = tk.Scale(self.root, from_=0, to=len(image_and_scans.get_scans())-1, 
                                     tickinterval=int(len(image_and_scans.get_scans())/5), orient=tk.HORIZONTAL, 
                                     command=self.scan_slider_callback)
        self.scan_start_slider.set(0)
        self.scan_start_slider.pack(ipadx=100)

        self.scan_end_slider = tk.Scale(self.root, from_=0, to=len(image_and_scans.get_scans())-1, 
                                     tickinterval=int(len(image_and_scans.get_scans())/5), orient=tk.HORIZONTAL, 
                                     command=self.scan_slider_callback)
        self.scan_end_slider.set(len(image_and_scans.get_scans())-1)
        self.scan_end_slider.pack(ipadx=100)
        

        # scan_start_slider.bind("<ButtonRelease-1>", lambda event : print("test"))
        
        self.button_frame = ttk.Frame(self.root)    
        self.button_frame.pack(side="top", pady=(20,0))

        self.select_points_button = ttk.Button(self.button_frame, text="Select Points")
        self.select_points_button.pack(side="left", padx=25,pady=(0,10), ipadx=20, ipady=20)
        self.select_points_button.bind("<ButtonPress>", self.select_points)


        self.done_button = ttk.Button(self.button_frame, text="Done")
        self.done_button.pack(side="left", padx=25,pady=(0,10), ipadx=20, ipady=20)
        self.done_button.bind("<ButtonPress>", self.done_callback)

        self.ax_lidar_points = None
        self.ax_selected_lidar_points = None
        self.scan_indices = np.arange(0,len(image_and_scans.get_scans()))
        self.selected_points_indices = None

        self.add_figure()

        self.reset_and_add_2d_lidar_points()

    
    def reset_and_add_2d_lidar_points(self):
        self.clear_2d_lidar_points()
        self.clear_selected_2d_lidar_points()
        self.all_2d_lidar_points = self.image_and_scans.concatenate_scans_to_points(self.scan_indices)
        self.add_2d_lidar_points(self.all_2d_lidar_points.copy())
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def scan_slider_callback(self,event):
        if self.scan_end_slider.get() < self.scan_start_slider.get():
            self.scan_end_slider.set(self.scan_start_slider.get())

        self.scan_indices = np.arange(self.scan_start_slider.get(),self.scan_end_slider.get()+1)
        
        self.reset_and_add_2d_lidar_points()

    

    def select_points(self, event):
        self.clear_selected_2d_lidar_points()

        points_xy = np.array([[point[0], point[1]] for point in self.all_2d_lidar_points.copy()])

        selected_x_indices = np.logical_and(points_xy[:,0]>= self.xlims[0], points_xy[:,0] <= self.xlims[1])
        selected_y_indices = np.logical_and(points_xy[:,1] >= self.ylims[0], points_xy[:,1] <= self.ylims[1])

        self.selected_points_indices = list(np.logical_and(selected_x_indices, selected_y_indices))
        points_xy_selected = points_xy[np.array(self.selected_points_indices)]


        self.ax_selected_lidar_points = self.ax.scatter(points_xy_selected[:,0], points_xy_selected[:,1], c='red')

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
    
    def done_callback(self, event):
        if self.selected_points_indices:
            pc2_points = self.image_and_scans.concatenate_scans_to_points(self.scan_indices)
            selected_pc2_points = pc2_points[self.selected_points_indices]
            self.image_and_scans.set_selected_lidar_points(selected_pc2_points)
            # print(len(self.image_and_scans.get_selected_lidar_points()))
            self.root.destroy()

    def on_xlims_change(self, event_ax):
        self.xlims = event_ax.get_xlim()
        # print('updated xlims:', self.xlims)

    def on_ylims_change(self, event_ax):
        self.ylims = event_ax.get_ylim()
        # self.clear_2d_lidar_points()
        # print('updated ylims:', self.ylims)


    def add_figure(self):

        self.figure = plt.Figure(figsize=(7, 5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.chart_type = FigureCanvasTkAgg(self.figure, self.root)
        self.navigation_tool_bar = tkagg.NavigationToolbar2Tk(self.chart_type, self.root)
        self.chart_type.get_tk_widget().pack()
        self.ax.set_title('2D LiDAR Scanned Points')


        self.ax.callbacks.connect('xlim_changed', self.on_xlims_change)
        self.ax.callbacks.connect('ylim_changed', self.on_ylims_change)
        # self.ax.set_axis_off()
        pass

    def clear_selected_2d_lidar_points(self):
        if self.ax_selected_lidar_points:
            self.ax_selected_lidar_points.remove()
        self.ax_selected_lidar_points = None
        self.selected_points_indices = None

    def clear_2d_lidar_points(self):
        if self.ax_lidar_points:
            self.ax_lidar_points.remove()
        self.ax_lidar_points = None
    
    def add_2d_lidar_points(self, points:np.ndarray):
        assert isinstance(points, np.ndarray), "Error: points must be a numpy array."
        points_xy = np.array([[point[0], point[1]] for point in points])
        self.ax_lidar_points = self.ax.scatter(points_xy[:,0], points_xy[:,1], c='blue')

    def run(self) -> ImageAndScans:
        self.app.mainloop()
        return self.image_and_scans
    
class CameraParameters:
    def __init__(self, camera_intrinsic_matrix, k1,k2,p1,p2,k3):
        assert isinstance(camera_intrinsic_matrix, np.ndarray), "Error: camera_intrinsic_matrix must be a matrix."
        assert camera_intrinsic_matrix.shape == (3,3), "Error: camera_intrinsic_matrix must be a 3x3 matrix."
        self.camera_intrinsic_matrix = camera_intrinsic_matrix
        self.distortion_coeffs = np.array([k1,k2,p1,p2,k3])
        
    def get_camera_parameters(self) -> tuple[np.ndarray, list]:
        return (self.camera_intrinsic_matrix.copy(), self.distortion_coeffs.copy())

class BagToImageAndScans(Node):

    def __init__(self):
        super().__init__('camera_2d_lidar_calibration')
        
        # Declare parameters with default values
        self.declare_parameter("camera_intrinsic_matrix", [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0])
        self.declare_parameter("distortion_coefficients.k1", 0.0)
        self.declare_parameter("distortion_coefficients.k2", 0.0)
        self.declare_parameter("distortion_coefficients.p1", 0.0)
        self.declare_parameter("distortion_coefficients.p2", 0.0)
        self.declare_parameter("distortion_coefficients.k3", 0.0)

        # Retrieve parameters
        camera_intrinsic_matrix = self.get_parameter("camera_intrinsic_matrix").value
        k1 = self.get_parameter("distortion_coefficients.k1").value
        k2 = self.get_parameter("distortion_coefficients.k2").value
        p1 = self.get_parameter("distortion_coefficients.p1").value
        p2 = self.get_parameter("distortion_coefficients.p2").value
        k3 = self.get_parameter("distortion_coefficients.k3").value

        self.camera_params = CameraParameters(np.array(camera_intrinsic_matrix).reshape(3,3), k1,k2,p1,p2,k3)

        self.image_and_scan_list = []

        self.image_publisher = self.create_publisher(Image, '/image', 10) # publisher for real-time image monitoring if necessary
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10) # publisher for real-time lidar monitoring if necessary

        bags_location = f'{home}/ros2_ws/src/camera_2d_lidar_calibration/bags/'
        
        for bag_file_name in os.listdir(bags_location):
            self.get_logger().info('Processing ' + bag_file_name + 'bag file.')
            self.reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(
                uri=f"{bags_location}{bag_file_name}/",
                storage_id='sqlite3')

            converter_options = rosbag2_py.ConverterOptions('', '')
            self.reader.open(storage_options, converter_options)

            self.image_and_scan_list.append(self.extract_image_and_scans())

    def extract_image_and_scans(self) -> ImageAndScans:
        selected_image = False
        image = None
        scans = []
        while self.reader.has_next():
            msg = self.reader.read_next()
            if msg[0] == '/camera/image_raw' and selected_image == False: # ignore other ROS topics in this bag
                decoded_data = deserialize_message(msg[1], Image) # get serialized version of message and decode it
                cvbridge = cv_bridge.CvBridge()
                cv_image = cvbridge.imgmsg_to_cv2(decoded_data, desired_encoding='passthrough') # change ROS2 Image to cv2 image
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR) # changes image encoding from RGB to BGR for cv2.imwrite to work correctly
                # cv2.imwrite(file_location, cv_image)
                image = cv_image
                selected_image = True
                self.image_publisher.publish(msg[1])
                self.get_logger().info(f'Published serialized data to /image')
            
            if msg[0] == '/scan':
                decoded_data = deserialize_message(msg[1], LaserScan) # get serialized version of message and decode it
                self.lidar_publisher.publish(msg[1])
                scans.append(decoded_data)      
        return ImageAndScans(image, scans)

    def get_camera_params(self) -> list[CameraParameters]:
        return self.camera_params
    
    def get_image_and_laser_scans(self) -> list[ImageAndScans]:
        return self.image_and_scan_list.copy()

def visualise_image(image:np.ndarray):
    fig = plt.figure()
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    plt.imshow(image)
    plt.show()

def get_vertical_and_horizontal_lines(img:np.ndarray):
    dst = cv2.Canny(img, 50, 200, None, 3)
    
    # print(img.shape)

    # Copy edges to the images that will display the results in BGR
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    vertical_gradient_min = 10 # minimum gradient to be considered a vertical line
    horizontal_gradient_max = 5 # maximum gradient to be considered a horizontal line

    vertical_lines = []
    horizontal_lines = []
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            m = (l[3] - l[1])/(l[2]-l[0]) if l[2]!=l[0] else np.inf
            if np.abs(m) >= vertical_gradient_min:
                vertical_lines.append([(l[0], l[1]), (l[2], l[3])])
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            if np.abs(m) <= horizontal_gradient_max:
                horizontal_lines.append([(l[0], l[1]), (l[2], l[3])])

    cv2.imshow("Source", img)
    cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    # cv2.waitKey()
    return (np.array(vertical_lines), np.array(horizontal_lines))

def get_detected_checkerboard(img:np.ndarray, camera_params:CameraParameters, chessboard_size=(11,8), square_size=2.0):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_size = (gray.shape[1], gray.shape[0])
    
    # Prepare object points, such as (0,0,0), (1,0,0), (2,0,0) ....,(10,7,0)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

    # Arrays to store object points and image points from all the images
    # objp = []  # 3D point in real world space
    corners2 = []  # 2D points in image plane


    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    # If corners found, refine and save them
    if ret == True:
        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        ret, rvec, tvec = cv2.solvePnP(objp, corners2, *camera_params.get_camera_parameters())

        R, _ = cv2.Rodrigues(rvec)

        # Full extrinsic matrix [R | t]
        extrinsic_matrix = np.vstack([np.hstack((R, tvec)),[0,0,0,1]])

        # print(corners2)
        # Save object and image points
        
        # Draw and display corners
        img_with_corners = img.copy()
        cv2.drawChessboardCorners(img_with_corners, chessboard_size, corners2, ret)
        
        cv2.imshow("Image with Corners",img_with_corners)

        # cv2.waitKey()

        # Create detection results directory
        # detection_dir = 'detection_results'
        # if not os.path.exists(detection_dir):
        #     os.makedirs(detection_dir)
            
        # # # Save image with drawn corners
        # # base_name = os.path.basename(fname)
        # # output_path = os.path.join(detection_dir, f"detected_{base_name}")
        # # cv2.imwrite(output_path, img_with_corners)
        return corners2.reshape(len(corners2),-1), objp, extrinsic_matrix
    else:
        return corners2, objp, extrinsic_matrix

def checkerboard_pixels_to_camera_frame(image_points, camera_params:CameraParameters, extrinsic_matrix):
    """
    Assumes the n checkerboard image points are given in units of pixels, with shape n x 2.
    Example: 
    image_points = np.array([[10, 20], [30,40]], dtype=np.float32)
    """
    K_inv = np.linalg.inv(camera_params.get_camera_parameters()[0])

    # Convert 2D image points to normalized camera coordinates
    image_points_h = np.hstack((image_points, np.ones((image_points.shape[0], 1))))  # (u, v, 1)
    camera_rays = (K_inv @ image_points_h.T).T  # Normalized direction vectors

    checkerboard_position = (extrinsic_matrix @ (np.array([0,0,0,1]).reshape(4,1)))[:3,:]
    checkerboard_x_vec = (extrinsic_matrix @ (np.array([1,0,0,1]).reshape(4,1)))[:3,:] - checkerboard_position
    checkerboard_y_vec = (extrinsic_matrix @ (np.array([0,1,0,1]).reshape(4,1)))[:3,:] - checkerboard_position
    
    projected_rays = []
    for ray in camera_rays:
        ray = ray.reshape(3,1)
        ray_trace_matrix = np.hstack([ray, checkerboard_x_vec, checkerboard_y_vec])
        coeffs = np.linalg.inv(ray_trace_matrix) @ checkerboard_position
        projected_ray = coeffs[0] * ray
        projected_rays.append(projected_ray.reshape(3))
    return np.array(projected_rays)

def camera_lidar_calibration(camera_params:CameraParameters, image_and_scan_list:list[ImageAndScans]):
    """
    Performs Camera and 2D LiDAR calibration by assuming that the edges of wall correspond to edges of the selected points in scan.
    """

    img = image_and_scan_list[0].get_image()

    h, w = img.shape[:2]
    newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(
    *camera_params.get_camera_parameters(), (w,h), 1, (w,h)
    )
    undistorted_image = cv2.undistort(
    img, *camera_params.get_camera_parameters(), None, newcameramatrix
    )
    img = undistorted_image
    # cv2.imshow("undistorted", undistorted_image)


    vertical_lines, horizontal_lines = get_vertical_and_horizontal_lines(img)
    print(vertical_lines)

    corners, corners_3d_world, extrinsic_matrix = get_detected_checkerboard(img, camera_params)
    # print(corners_3d_world)
    corners_camera_frame = np.array([((extrinsic_matrix @ np.vstack([corner_3d.reshape(3,1),[1]]))[:3,:]).reshape(3) for corner_3d in corners_3d_world])
    
    
    print(corners_camera_frame)

    vertical_lines_camera_frame = []
    for vertical_line in vertical_lines:
        vertical_lines_camera_frame.append(checkerboard_pixels_to_camera_frame(vertical_line, camera_params, extrinsic_matrix))
    vertical_lines_camera_frame = np.array(vertical_lines_camera_frame)
    


    # print(checkerboard_pixels_to_camera_frame())




        
        

    # cv2.waitKey()


    
    # print(corners_projected.max(axis=0) - corners_projected.min(axis=0))

    # cv2.projectPoints()

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(corners_camera_frame[:,0],corners_camera_frame[:,1], corners_camera_frame[:,2])
    for line in vertical_lines_camera_frame:
        ax.plot(line[:,0],line[:,1], line[:,2])
    ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])

    # plt.plot()
    plt.show()

    # edges = cv2.Canny(img,100,200)

    
    # return 0


    # visualise_image()
    

    pass



def main(args=None):
    try:
        rclpy.init(args=args)
        bag_to_image_and_scans = BagToImageAndScans()
        
        camera_params = bag_to_image_and_scans.get_camera_params()
        image_and_scan_list = bag_to_image_and_scans.get_image_and_laser_scans()
        
        updated_image_and_scan_list = []

        for image_and_scan in image_and_scan_list:
            select_points_interface = SelectPointsInterface(image_and_scan)
            image_and_scan = select_points_interface.run()
            updated_image_and_scan_list.append(image_and_scan)

        camera_lidar_calibration(camera_params,updated_image_and_scan_list)


        # rclpy.spin(bag_to_image_and_scans)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()