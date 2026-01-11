#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from custom_msg_pkg.msg import SensorReading
import random
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Publisher 생성
        self.publisher_ = self.create_publisher(
            SensorReading,
            'sensor_data',
            10  # QoS depth
        )
        
        # 타이머 설정 (1초마다 발행)
        self.timer = self.create_timer(1.0, self.publish_sensor_data)
        
        # 센서 ID 초기화
        self.sensor_id = 1
        
        self.get_logger().info('Sensor Publisher 시작됨')
    
    def publish_sensor_data(self):
        msg = SensorReading()
        
        # 기본 헤더 정보 설정
        msg.sensor_id = self.sensor_id
        current_time = self.get_clock().now().to_msg()
        msg.timestamp = current_time
        
        # 위치 정보 설정 (랜덤 값)
        msg.position = Point()
        msg.position.x = random.uniform(-10.0, 10.0)
        msg.position.y = random.uniform(-10.0, 10.0)
        msg.position.z = random.uniform(0.0, 5.0)
        
        # 센서 측정값 설정
        msg.temperature = random.uniform(20.0, 35.0)
        msg.humidity = random.uniform(30.0, 80.0)
        msg.pressure = random.uniform(1000.0, 1020.0)
        
        # 상태 플래그 설정
        msg.is_active = True
        msg.has_error = random.choice([True, False])
        
        # 메타데이터 설정
        msg.sensor_name = f"Sensor_{self.sensor_id}"
        msg.location = "Laboratory_A"
        
        # 메시지 발행
        self.publisher_.publish(msg)
        
        self.get_logger().info(
            f'센서 데이터 발행: ID={msg.sensor_id}, '
            f'온도={msg.temperature:.2f}°C, '
            f'습도={msg.humidity:.2f}%'
        )

def main(args=None):
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()