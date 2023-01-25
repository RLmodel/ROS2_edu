import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtlesimSubscriber(Node):
    def __init__(self):
        super().__init__('turtlesim_subscriber')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)

    def callback(self, msg):
        # print("X: ", msg.x, "Y: ", msg.y)
                print("turtle pose X: ", "{:.2f}".format(msg.x),
                " / Y: ", "{:.2f}".format(msg.y),
                " Yaw: ", "{:.2f}".format(msg.theta)
                )
        # print("turtle angle Yaw: ", "{:.2f}".format(msg.theta))

def main(args=None):
        rp.init(args=args)

        turtlesim_subscriber = TurtlesimSubscriber()
        rp.spin(turtlesim_subscriber)

        turtlesim_subscriber.destroy_node()
        rp.shutdown()

if __name__=='__main__':
    main()

    