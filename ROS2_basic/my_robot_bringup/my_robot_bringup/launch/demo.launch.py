from launch import LaunchDescription
from launch_ros.actions import Node

def generate_lanch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="toy",
        remappings=[
            ("/turtle1/cmd_vel", "/cmd_vel"),
            ("/clear", "/cls")
        ]
        parameters=[
            {"background_r": 255},
            {"background_g": 0},
            {"background_b": 0}
        ]
    )

    ld.add_action(turtlesim_node)
    return ld
