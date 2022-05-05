from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mag_pl_detector'),
        'config',
        'params.yaml'
    )

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[config]
    )

    physical_setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mag_pl_detector"),
                "launch/physical_setup.launch.py"
            ])
        ])
    )

    mag_sample_publisher = Node(
        package="mag_pl_detector",
        executable="mag_sample_publisher",
        parameters = [config]
    )

    sine_reconstructor = Node(
        package="mag_pl_detector",
        executable="sine_reconstructor",
        parameters=[config]
    )

    pl_dir_computer = Node(
        package="mag_pl_detector",
        executable="pl_dir_computer",
        parameters=[config]
    )

    pl_dir_estimator = Node(
        package="mag_pl_detector",
        executable="pl_dir_estimator",
        parameters=[config]
    )

    return LaunchDescription([
        camera,
        physical_setup_launch,
        mag_sample_publisher,
        sine_reconstructor,
        pl_dir_computer,
        pl_dir_estimator,
    ])
