from rclpy.node import Node
from std_msgs.msg import String
import rclpy

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)

        self.timer = self.create_timer(1.0, self.publish_message)

        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'안녕하세요! 메시지 번호: {self.counter}'

        self.publisher_.publish(msg)

        self.get_logger().info(f'발행된 메시지: {msg.data}')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()