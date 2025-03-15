import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rosbag2_py
from sensor_msgs.msg import Image, LaserScan
import os
from rclpy.serialization import deserialize_message
import cv2
import cv_bridge
from pathlib import Path
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import laser_geometry
home = Path.home()

class ImageAndScans:
    def __init__(self, image:np.ndarray, scans:list[LaserScan]):
        assert isinstance(image, np.ndarray), "Error: image must be of the form cv2.Mat."
        assert isinstance(scans, list), "Error: scans must be a list."
        for scan in scans:
            assert isinstance(scan, LaserScan), "Error: scans must contain LaserScan objects."
        self.image = image
        self.scans = scans

    def concatenate_scans_to_points(self) -> np.ndarray:
        scans = self.get_scans()
        laser_projection = laser_geometry.LaserProjection()
        projected_scans = [laser_projection.projectLaser(scan) for scan in scans]
        scans_numpy = [pc2.read_points(scan) for scan in projected_scans]
        concatenated_scans = np.concatenate(scans_numpy)
        return concatenated_scans # x,y,z,intensity,index

    def get_image(self) -> np.ndarray:
        return self.image.copy()
    def get_scans(self) -> list[LaserScan]:
        return self.scans.copy()
    
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
    

def main(args=None):
    try:
        rclpy.init(args=args)
        bag_to_image_and_scans = BagToImageAndScans()
        
        camera_params = bag_to_image_and_scans.get_camera_params()
        image_and_scan_list = bag_to_image_and_scans.get_image_and_laser_scans()

        print(image_and_scan_list[0].concatenate_scans_to_points())

        # rclpy.spin(bag_to_image_and_scans)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()