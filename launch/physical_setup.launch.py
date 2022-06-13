from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tf_drone_to_mag0 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.1925", "-0.1925", "0","0.7854", "0", "0",  "drone", "mag0"]
    )

    tf_drone_to_mag1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.1925", "-0.1925", "0", "2.3562", "0", "0",  "drone", "mag1"]
    )

    tf_drone_to_mag2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.1925", "0.1925", "0", "-2.3562", "0", "0", "drone", "mag2"]
    )

    tf_drone_to_mag3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.1925", "0.1925", "0", "-0.7854", "0", "0",  "drone", "mag3"]
    )

    world_to_drone = Node(
        package="iii_drone",
        executable="drone_frame_broadcaster"
    )

    return LaunchDescription([
        world_to_drone,
		tf_drone_to_mag0,
		tf_drone_to_mag1,
		tf_drone_to_mag2,
		tf_drone_to_mag3,
    ])
