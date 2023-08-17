from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    imgpub_node = Node(
        package="cv_basics",
        executable="img_publisher",
    )
    
    imgsub_node = Node(
        package="cv_basics",
        executable="img_subscriber"
    )
  
    ld.add_action(imgpub_node)
    ld.add_action(imgsub_node)

    return ld