from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_strpubsub',
            namespace='ns1',
            executable='first_pub',
            name='my_publisher_node'
        ),
        Node(
            package='py_strpubsub',
            namespace='ns1',
            executable='first_sub',
            name='my_subscriber_node'
        )
    ])
