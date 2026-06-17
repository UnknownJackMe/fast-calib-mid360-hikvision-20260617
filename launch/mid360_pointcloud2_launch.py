import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fast_calib_share = get_package_share_directory("fast_calib")
    user_config_path = os.path.join(fast_calib_share, "config", "livox_mid360_fast_calib.json")

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"lvx_file_path": "/home/livox/livox_test.lvx"},
            {"user_config_path": user_config_path},
            {"cmdline_input_bd_code": "livox0000000001"},
        ],
    )

    return LaunchDescription([livox_driver])
