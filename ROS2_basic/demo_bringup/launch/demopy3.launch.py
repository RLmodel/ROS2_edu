from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="demo_talker",
        remappings=[
            ("chatter", "demo_chatter")
        ],
        parameters=[
            {"param_name": "value"},
            {"param2": 20}
        ]
    )

    listner_node = Node(
        package="demo_nodes_py",
        executable="listener",
        name="demo_listener",
        remappings=[
            ("chatter", "demo_chatter")
        ]
    )

    ld.add_action(talker_node)
    ld.add_action(listner_node)

    return ld