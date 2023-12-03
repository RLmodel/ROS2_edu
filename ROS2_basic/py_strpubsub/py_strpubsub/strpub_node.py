# ROS Client Library for Python
import rclpy
 
# Handles the creation of nodes
from rclpy.node import Node
 
# Enables usage of the String message type
from std_msgs.msg import String
 
class StrPub(Node):
  """
  Create a StrPub class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('str_pub')
     
    # Create the publisher. This publisher will publish a String message
    # to the addison topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(String, 'rlmodel', 10)
     
    # We will publish a message every 0.5 seconds
    timer_period = 0.5  # seconds
     
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
  
    # Initialize a counter variable
    self.i = 0
 
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.5 seconds.
    """
    # Create a String message
    msg = String()
 
    # Set the String message's data
    msg.data = 'hello ros2: %d' % self.i
     
    # Publish the message to the topic
    self.publisher_.publish(msg)
     
    # Display the message on the console
    self.get_logger().info('Publishing: "%s"' % msg.data)
 
    # Increment the counter by 1    
    self.i += 1
 
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create the node
  str_pub = StrPub()
 
  # Spin the node so the callback function is called.
  rclpy.spin(str_pub)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  str_pub.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()