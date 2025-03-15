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
import time
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
        self.distortion_coeffs = [k1,k2,p1,p2,k3]
        
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



def camera_lidar_calibration(camera_params:CameraParameters, image_and_scan_list:list[ImageAndScans]):
    """
    Performs Camera and 2D LiDAR calibration by assuming that the edges of wall corresopnd to edges of the selected points in scan.
    """
    
    pass



def main(args=None):
    try:
        rclpy.init(args=args)
        bag_to_image_and_scans = BagToImageAndScans()
        
        camera_params = bag_to_image_and_scans.get_camera_params()
        image_and_scan_list = bag_to_image_and_scans.get_image_and_laser_scans()
        
        for image_and_scan in image_and_scan_list:
            select_points_interface = SelectPointsInterface(image_and_scan)
            image_and_scan = select_points_interface.run()
            
            print(len(image_and_scan.get_selected_lidar_points()))



        # rclpy.spin(bag_to_image_and_scans)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()