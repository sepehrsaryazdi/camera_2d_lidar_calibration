import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rosbag2_py
from sensor_msgs.msg import Image
import os
from rclpy.serialization import deserialize_message
import cv2
import cv_bridge
from pathlib import Path
from rcl_interfaces.msg import SetParametersResult

home = Path.home()

class BagToImage(Node):

    def __init__(self):
        super().__init__('camera_2d_lidar_calibration')
        
        # Declare parameters with default values
        self.declare_parameter("camera_intrinsic_matrix", [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0])

        # Retrieve parameters
        camera_intrinsic_matrix = self.get_parameter("camera_intrinsic_matrix").value



        self.publisher = self.create_publisher(Image, '/image', 10) # publisher for real-time monitoring if necessary

        bags_location = f'{home}/ros2_ws/src/camera_2d_lidar_calibration/bags/'
        images_location = f'{home}/ros2_ws/src/camera_2d_lidar_calibration/images/'
        if not os.path.exists(images_location):
            os.mkdir(images_location)
        for bag_file_name in os.listdir(bags_location):
            self.get_logger().info('Processing ' + bag_file_name + 'bag file.')
            self.reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(
                uri=f"{bags_location}{bag_file_name}/",
                storage_id='sqlite3')

            converter_options = rosbag2_py.ConverterOptions('', '')
            self.reader.open(storage_options, converter_options)

            save_image_location = images_location + bag_file_name + '/'
            if not os.path.exists(save_image_location):
                os.mkdir(save_image_location)
            self.extract_image(save_image_location)
        
    def extract_image(self, save_image_location):
        image_counter = 0
        while self.reader.has_next():
            msg = self.reader.read_next()
            if msg[0] != '/camera/image_raw': # ignore other ROS topics in this bag
                continue
            decoded_data = deserialize_message(msg[1], Image) # get serialized version of message and decode it
            cvbridge = cv_bridge.CvBridge()
            cv_image = cvbridge.imgmsg_to_cv2(decoded_data, desired_encoding='passthrough') # change ROS2 Image to cv2 image
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR) # changes image encoding from RGB to BGR for cv2.imwrite to work correctly
            file_location = save_image_location + str(image_counter) + ".png"
            cv2.imwrite(file_location, cv_image)
            self.publisher.publish(msg[1])
            self.get_logger().info(f'Saved {file_location} and published serialized data to /image')
            image_counter+=1

def main(args=None):
    try:
        rclpy.init(args=args)
        sbr = BagToImage()
        rclpy.spin(sbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()