#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.counter = 0
        
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # 경로를 나타내는 선 마커
        line_marker = Marker()
        line_marker.header.frame_id = 'base_link'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.1 # 선의 두깨
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        # 나선형 경로 생성
        for i in range(100):
            point = Point()
            angle = i * 0.1
            point.x = angle * 0.1 * math.cos(angle)
            point.y = angle * 0.1 * math.sin(angle)
            point.z = i * 0.02
            line_marker.points.append(point)

        marker_array.markers.append(line_marker)
        
        # 목표점을 나타내는 화살표 마커
        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'base_link'
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        arrow_marker.pose.position.x = 3.0
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 1.0
        arrow_marker.pose.orientation.w = 1.0

        arrow_marker.scale.x = 1.0 # 화살표 길이
        arrow_marker.scale.y = 0.1 # 화살표 폭
        arrow_marker.scale.z = 0.1 # 화살표 높이

        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        marker_array.markers.append(arrow_marker)

        self.publisher.publish(marker_array)

def main(args=None):
    # main 진입 함수 정의
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()

    try:
        rclpy.spin(marker_publisher)
    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때 ROS 2 로거를 통해 종료 메시지를 출력합니다.
        marker_publisher.get_logger().info('Marker Publisher node stopped by KeyboardInterrupt')
    finally:
        if rclpy.ok():
            marker_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()