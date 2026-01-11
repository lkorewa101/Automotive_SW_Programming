#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msg_pkg.msg import SensorReading
import math

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        # Subscriber 생성
        self.subscription = self.create_subscription(
            SensorReading,
            'sensor_data',
            self.sensor_callback,
            10  # QoS depth
        )
        
        # 데이터 분석을 위한 변수들
        self.message_count = 0
        self.temperature_sum = 0.0
        self.humidity_sum = 0.0
        self.error_count = 0
        
        self.get_logger().info('Sensor Subscriber 시작됨')
    
    def sensor_callback(self, msg):
        """센서 데이터 수신 콜백 함수"""
        self.message_count += 1
        
        # 기본 정보 로깅
        self.get_logger().info(
            f'센서 데이터 수신 #{self.message_count}: '
            f'ID={msg.sensor_id}, 이름={msg.sensor_name}'
        )
        
        # 타임스탬프 분석
        timestamp_sec = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        self.get_logger().info(f'타임스탬프: {timestamp_sec:.3f}초')
        
        # 위치 정보 분석
        distance = math.sqrt(
            msg.position.x**2 + msg.position.y**2 + msg.position.z**2
        )
        self.get_logger().info(
            f'위치: ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}), '
            f'원점거리: {distance:.2f}m'
        )
        
        # 센서 측정값 분석
        self.temperature_sum += msg.temperature
        self.humidity_sum += msg.humidity
        avg_temp = self.temperature_sum / self.message_count
        avg_humidity = self.humidity_sum / self.message_count
        
        self.get_logger().info(
            f'측정값 - 온도: {msg.temperature:.2f}°C (평균: {avg_temp:.2f}°C), '
            f'습도: {msg.humidity:.2f}% (평균: {avg_humidity:.2f}%), '
            f'압력: {msg.pressure:.2f}hPa'
        )
        
        # 상태 분석
        if msg.has_error:
            self.error_count += 1
            self.get_logger().warn(
                f'센서 오류 감지! (총 오류 횟수: {self.error_count})'
            )
        
        status = "활성" if msg.is_active else "비활성"
        self.get_logger().info(f'센서 상태: {status}, 위치: {msg.location}')
        
        # 임계값 검사
        self.check_sensor_thresholds(msg)
        
        self.get_logger().info('-' * 50)
    
    def check_sensor_thresholds(self, msg):
        """센서 값의 임계값 검사"""
        warnings = []
        
        if msg.temperature > 30.0:
            warnings.append(f"고온 경고: {msg.temperature:.2f}°C")
        elif msg.temperature < 22.0:
            warnings.append(f"저온 경고: {msg.temperature:.2f}°C")
            
        if msg.humidity > 75.0:
            warnings.append(f"고습도 경고: {msg.humidity:.2f}%")
        elif msg.humidity < 35.0:
            warnings.append(f"저습도 경고: {msg.humidity:.2f}%")
            
        if msg.pressure > 1018.0 or msg.pressure < 1002.0:
            warnings.append(f"기압 이상: {msg.pressure:.2f}hPa")
        
        for warning in warnings:
            self.get_logger().warn(warning)

def main(args=None):
    rclpy.init(args=args)
    
    sensor_subscriber = SensorSubscriber()
    
    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()