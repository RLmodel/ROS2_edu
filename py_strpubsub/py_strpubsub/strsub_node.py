# ROS Client Library for Python
import rclpy
 
# Handles the creation of nodes
from rclpy.node import Node
 
# Handles string messages
from std_msgs.msg import String
 
class MinimalSubscriber(Node):
  """
  Create a subscriber node
  """
  def __init__(self):
 
    # Initiate the Node class's constructor and give it a name
    super().__init__('minimal_subscriber')
 
    # The node subscribes to messages of type std_msgs/String, 
    # over a topic named: /addison
    # The callback function is called as soon as a message is received.
    # The maximum number of queued messages is 10.
    self.subscription = self.create_subscription(
      String,
      'rlmodel',
      self.listener_callback,
      10)
    self.subscription  # prevent unused variable warning
 
  def listener_callback(self, msg):
    # Display a message on the console every time a message is received on the
    # addison topic
    self.get_logger().info('I heard: "%s"' % msg.data)
 
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create a subscriber
  minimal_subscriber = MinimalSubscriber()
 
  # Spin the node so the callback function is called.
  # Pull messages from any topics this node is subscribed to.
  rclpy.spin(minimal_subscriber)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  minimal_subscriber.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()