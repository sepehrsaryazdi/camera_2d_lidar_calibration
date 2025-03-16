from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_2d_lidar_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # Install config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools', 'opencv-python','tk', 'matplotlib','scikit-learn','numpy<2.0'],
    zip_safe=True,
    maintainer='sepehr',
    maintainer_email='sepehr.saryazdi@gmail.com',
    description='ROS2 Bag to Camera and 2D LiDAR Calibration Package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_2d_lidar_calibration = camera_2d_lidar_calibration.camera_2d_lidar_calibration:main',
        ],
    },
)