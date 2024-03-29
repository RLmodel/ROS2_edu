from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
#from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    demo_launch_file = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(
        #XMLLaunchDescriptionSource(
        YAMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('demo_bringup'),
                         'launch/demoyaml1.launch.yaml')
        )
    )

    demo_launch_file2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        #XMLLaunchDescriptionSource(
        #YAMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('demo_bringup'),
                         'launch/demopy1.launch.py')
        )
    )

    ld.add_action(demo_launch_file)
    ld.add_action(demo_launch_file2)
    return ld