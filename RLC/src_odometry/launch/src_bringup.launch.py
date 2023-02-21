import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    mw_ahrs_node = Node(
        package='mw_ahrsv1_ros2',
        executable='mw_ahrsv1_only_angle',
        name='mw_ahrsv1_ros2',
        output='log',
        parameters=[{
            'deviceID' : '/dev/MWAHRs',
            'frame_id' : 'base_link',
            'child_frame_id' : 'imu_link',
            'publish_tf' : True,
            'view_imu' : False,
            'verbose' : False,
            'publish_rate' : 50,
        }],
    )

    src_odom = Node(
        package='src_odometry',
        executable='src_odometry',
        name='src_odometry',
        output='log',
        parameters=[{
            "verbose" : False,
            'publish_rate' : 50,
            'open_loop' : False,
            'has_imu_heading' : True,
            'is_gazebo' : False,
            'wheel_radius' : 0.0508,
            'base_frame_id' : "base_link",
            'odom_frame_id' : "odom",
            'enable_odom_tf' : True,
        }],
    )

    cmd_to_src_pkg = os.path.join(get_package_share_directory('cmd_to_src'))
    cmd_to_src = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cmd_to_src_pkg, 'launch', 'cmd_to_src.launch.py')),
    )

    return LaunchDescription([
        mw_ahrs_node,
        src_odom,
        cmd_to_src,
    ])