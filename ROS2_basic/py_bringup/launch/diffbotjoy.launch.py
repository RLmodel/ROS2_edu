from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
#from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joy",
        executable="joy_node",
    )

    joytele_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
            remappings=[
                ("/cmd_vel", "/diffbot/cmd_vel")
        ],
    )

    launch_file2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        #XMLLaunchDescriptionSource(
        #YAMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gcamp_gazebo'),
                         'launch/maze_world.launch.py')
        )
    )

    ld.add_action(joystick_node)
    ld.add_action(joytele_node)
    ld.add_action(launch_file2)
    
    return ld