#!/usr/bin/env python3

import rclpy
import rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init_(self):

        super().__init__('simple_publisher')

        self.publisher= self.create_publisher(String, '/hello_topic', 10)

        self.message_count = 0

        self.timer = self.create_timer(1.0, self,publish_message)

        self.get_logger().info('Simple Publisher 노드가 시작되었습니다.')
        self.get_logger().info('메시지를 /hello_topic에 발행합니다.')

    def publish_message(self):
        
        msg = String()

        msg.data = f'안녕하세요! 메시지 번호: {self.message_count}'

        self.publisher.publish(msg)

        self.get_logger().info(f'발행된 메시지: "{msg.data}"')

        self.message_count += 1

def main(args=None):
    rclpy.init(args=args)

    node = SimplePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        