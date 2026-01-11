#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import numpy as np

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.timer = self.create_timer(0.1, self.publish_pointcloud)
        self.counter = 0
        
    def publish_pointcloud(self):
        # PointCloud2 메시지 생성
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'sensor_frame'
        
        # 포인트 필드 정의
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 원통 형태의 포인트 클라우드 생성
        points = []
        for i in range(360):
            angle = math.radians(i)
            for height in np.arange(0, 2, 0.1):
                x = 1.5 * math.cos(angle)
                y = 1.5 * math.sin(angle)
                z = height
                intensity = height * 50  # 높이에 따른 강도 변화
                
                # float 데이터를 bytes로 변환
                point_data = struct.pack('ffff', x, y, z, intensity)
                points.append(point_data)
        
        # 포인트 클라우드 메시지 완성
        cloud_msg.data = b''.join(points)
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.point_step = 16  # 한 포인트당 바이트 수
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        self.publisher.publish(cloud_msg)
        self.counter += 1


def main(args=None):
    # main 진입 함수 정의
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()

    try:
        rclpy.spin(pointcloud_publisher)
    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때 ROS 2 로거를 통해 종료 메시지를 출력합니다.
        pointcloud_publisher.get_logger().info('Marker Publisher node stopped by KeyboardInterrupt')
    finally:
        if rclpy.ok():
            pointcloud_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()