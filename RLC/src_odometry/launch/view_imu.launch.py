import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Rviz
    src_odometry_pkg = os.path.join(get_package_share_directory('src_odometry'))
    rviz_config_dir = os.path.join(src_odometry_pkg, 'rviz', 'real_odom.rviz')

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )

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
            'verbose' : True,
            'publish_rate' : 50,
        }],
    )

    src_odom = Node(
        package='src_odometry',
        executable='src_odom',
        name='src_odom',
        output='log',
        parameters=[{
            'imu_topic_name' : 'imu/data',
            'encoder_topic_name' : '/encoder_value',
            'publish_tf' : True,
            'update_rate' : 50,
            'base_frame_id' : "base_link",
            'odom_frame_id' : "odom",
            'wheel_radius' : 0.0508,
            "encoder_resolution" : 150,
            "verbose" : False,
        }],
    )

    return LaunchDescription([
        mw_ahrs_node,
        src_odom,
        
        TimerAction(    
            period=2.0,
            actions=[rviz2]
        ),
    ])