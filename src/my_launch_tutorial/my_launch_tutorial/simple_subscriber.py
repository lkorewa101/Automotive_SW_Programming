from rclpy.node import Node
from std_msgs.msg import String
import rclpy

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.create_subscription(String, 'hello_topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'수신한 메시지: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
