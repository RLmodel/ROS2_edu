from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
#from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    demo_launch_file = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('demo_bringup'),
                         'launch/demoxml1.launch.xml')
        )
    )

    ld.add_action(demo_launch_file)
    return ld