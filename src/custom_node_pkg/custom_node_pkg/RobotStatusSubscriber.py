#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msg_pkg.msg import RobotStatus
import math

class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_subscriber')
        
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.robot_status_callback,
            10
        )
        
        # 상태 추적 변수들
        self.previous_status = None
        self.status_change_count = 0
        self.total_distance = 0.0
        self.previous_position = None
        
        # 상태명 매핑
        self.status_names = {
            RobotStatus.STATUS_IDLE: "대기중",
            RobotStatus.STATUS_MOVING: "이동중",
            RobotStatus.STATUS_ERROR: "오류",
            RobotStatus.STATUS_CHARGING: "충전중"
        }
        
        self.get_logger().info('Robot Status Subscriber 시작됨')
    
    def robot_status_callback(self, msg):
        """로봇 상태 메시지 처리 콜백"""
        
        # 헤더 정보 분석
        self.get_logger().info(f'프레임 ID: {msg.header.frame_id}')
        
        # 상태 변경 감지
        current_status_name = self.status_names.get(msg.status, "알 수 없음")
        if self.previous_status != msg.status:
            self.status_change_count += 1
            self.get_logger().info(
                f'상태 변경 감지 #{self.status_change_count}: '
                f'{self.status_names.get(self.previous_status, "초기")} → {current_status_name}'
            )
            self.previous_status = msg.status
        
        # 기본 상태 정보
        self.get_logger().info(
            f'현재 상태: {current_status_name}, '
            f'배터리: {msg.battery_level:.1f}%, '
            f'속도: {msg.speed:.2f}m/s'
        )
        
        # 위치 정보 분석 및 이동 거리 계산
        current_pos = msg.current_pose.position
        if self.previous_position is not None:
            distance = math.sqrt(
                (current_pos.x - self.previous_position.x)**2 +
                (current_pos.y - self.previous_position.y)**2 +
                (current_pos.z - self.previous_position.z)**2
            )
            self.total_distance += distance
            
        self.get_logger().info(
            f'위치: ({current_pos.x:.2f}, {current_pos.y:.2f}, {current_pos.z:.2f}), '
            f'총 이동거리: {self.total_distance:.2f}m'
        )
        
        # 방향 정보 (쿼터니언 → 오일러각 변환)
        orientation = msg.current_pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )
        yaw_degrees = math.degrees(yaw)
        self.get_logger().info(f'방향: {yaw_degrees:.1f}도')
        
        # 속도 정보 분석
        velocity = msg.current_velocity
        linear_speed = math.sqrt(
            velocity.linear.x**2 + velocity.linear.y**2 + velocity.linear.z**2
        )
        self.get_logger().info(
            f'선속도: {linear_speed:.2f}m/s, '
            f'각속도: {velocity.angular.z:.2f}rad/s'
        )
        
        # 센서 데이터 배열 분석
        self.analyze_sensor_array(msg.sensor_readings)
        
        # 오류 정보 분석
        self.analyze_error_status(msg.error_flags, msg.error_messages)
        
        # 현재 위치 저장
        self.previous_position = current_pos
        
        self.get_logger().info('=' * 60)
    
    def analyze_sensor_array(self, sensor_readings):
        """센서 데이터 배열 분석"""
        sensor_count = len(sensor_readings)
        self.get_logger().info(f'연결된 센서 수: {sensor_count}')
        
        if sensor_count == 0:
            self.get_logger().warn('센서 데이터가 없습니다!')
            return
        
        # 센서별 상세 정보
        total_temp = 0.0
        active_sensors = 0
        error_sensors = 0
        
        for i, sensor in enumerate(sensor_readings):
            total_temp += sensor.temperature
            if sensor.is_active:
                active_sensors += 1
            if sensor.has_error:
                error_sensors += 1
                
            self.get_logger().info(
                f'  센서 {sensor.sensor_id} ({sensor.sensor_name}): '
                f'온도={sensor.temperature:.1f}°C, '
                f'위치=({sensor.position.x:.1f}, {sensor.position.y:.1f}, {sensor.position.z:.1f}), '
                f'상태={"활성" if sensor.is_active else "비활성"}'
            )
        
        # 센서 통계
        avg_temp = total_temp / sensor_count
        self.get_logger().info(
            f'센서 통계: 평균온도={avg_temp:.1f}°C, '
            f'활성센서={active_sensors}/{sensor_count}, '
            f'오류센서={error_sensors}/{sensor_count}'
        )
    
    def analyze_error_status(self, error_flags, error_messages):
        """오류 상태 분석"""
        active_errors = sum(error_flags)
        
        if active_errors > 0:
            self.get_logger().warn(f'활성 오류 수: {active_errors}')
            
            # 오류 플래그 상세 분석
            for i, flag in enumerate(error_flags):
                if flag:
                    self.get_logger().warn(f'  시스템 {i+1}: 오류 상태')
            
            # 오류 메시지 출력
            for msg in error_messages:
                self.get_logger().warn(f'  오류 메시지: {msg}')
        else:
            self.get_logger().info('시스템 정상 상태')

def main(args=None):
    rclpy.init(args=args)
    
    robot_status_subscriber = RobotStatusSubscriber()
    
    try:
        rclpy.spin(robot_status_subscriber)
    except KeyboardInterrupt:
        pass
    
    robot_status_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()