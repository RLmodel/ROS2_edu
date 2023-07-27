from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
    )

    listner_node = Node(
        package="demo_nodes_py",
        executable="listener"
    )

    ld.add_action(talker_node)
    ld.add_action(listner_node)

    return ld