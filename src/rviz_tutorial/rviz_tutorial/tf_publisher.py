#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)
        self.angle = 0.0
        
    def publish_tf(self):
        # base_link에서 sensor_frame으로의 변환
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'sensor_frame'
        
        # 원형 운동 구현
        t.transform.translation.x = 2.0 * math.cos(self.angle)
        t.transform.translation.y = 2.0 * math.sin(self.angle)
        t.transform.translation.z = 1.0
        
        # 회전값 설정 (쿼터니언)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.angle / 2.0)
        t.transform.rotation.w = math.cos(self.angle / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        self.angle += 0.05


def main(args=None):
    # main 진입 함수 정의
    rclpy.init(args=args)
    tf_publisher = TFPublisher()

    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때 ROS 2 로거를 통해 종료 메시지를 출력합니다.
        tf_publisher.get_logger().info('TF Publisher node stopped by KeyboardInterrupt')
    finally:
        if rclpy.ok():
            tf_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()