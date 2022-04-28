from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    physical_setup_launch = IncludeLaunchDescription(
        package="mag_pl_detector",
        launch="physical_setup.launch.py"
    )

    mag_sample_publisher = Node(
        package="mag_pl_detector",
        executable="mag_sample_publisher"
    )

    sine_reconstructor = Node(
        package="mag_pl_detector",
        executable="sine_reconstructor"
    )

    return LaunchDescription([
        physical_setup_launch,
        mag_sample_publisher,
        sine_reconstructor
    ])
