#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from custom_msg_pkg.msg import RobotStatus, SensorReading
import random
import math

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        
        self.publisher_ = self.create_publisher(
            RobotStatus,
            'robot_status',
            10
        )
        
        self.timer = self.create_timer(2.0, self.publish_robot_status)
        
        # 로봇 상태 변수들
        self.current_status = RobotStatus.STATUS_IDLE
        self.battery_level = 100.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.status_counter = 0
        
        self.get_logger().info('Robot Status Publisher 시작됨')
    
    def publish_robot_status(self):
        msg = RobotStatus()
        
        # 헤더 설정
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # 상태 업데이트 시뮬레이션
        self.update_robot_state()
        
        # 기본 상태 정보
        msg.status = self.current_status
        msg.battery_level = self.battery_level
        msg.speed = random.uniform(0.0, 2.0) if self.current_status == RobotStatus.STATUS_MOVING else 0.0
        
        # 센서 데이터 배열 생성
        sensor_count = random.randint(2, 5)
        msg.sensor_readings = []
        for i in range(sensor_count):
            sensor_msg = SensorReading()
            sensor_msg.sensor_id = i + 1
            sensor_msg.timestamp = self.get_clock().now().to_msg()
            
            sensor_msg.position = Point()
            sensor_msg.position.x = float(i * 0.5)
            sensor_msg.position.y = float(i * 0.3)
            sensor_msg.position.z = 0.1
            
            sensor_msg.temperature = random.uniform(18.0, 30.0)
            sensor_msg.humidity = random.uniform(40.0, 70.0)
            sensor_msg.pressure = random.uniform(1005.0, 1015.0)
            sensor_msg.is_active = True
            sensor_msg.has_error = False
            sensor_msg.sensor_name = f"Robot_Sensor_{i+1}"
            sensor_msg.location = "Robot_Body"
            
            msg.sensor_readings.append(sensor_msg)
        
        # 위치 및 방향 정보
        msg.current_pose = Pose()
        msg.current_pose.position = Point()
        msg.current_pose.position.x = self.position_x
        msg.current_pose.position.y = self.position_y
        msg.current_pose.position.z = 0.0
        
        # 쿼터니언 설정 (Z축 회전)
        yaw = random.uniform(-math.pi, math.pi)
        msg.current_pose.orientation = Quaternion()
        msg.current_pose.orientation.x = 0.0
        msg.current_pose.orientation.y = 0.0
        msg.current_pose.orientation.z = math.sin(yaw / 2.0)
        msg.current_pose.orientation.w = math.cos(yaw / 2.0)
        
        # 속도 정보
        msg.current_velocity = Twist()
        if self.current_status == RobotStatus.STATUS_MOVING:
            msg.current_velocity.linear = Vector3()
            msg.current_velocity.linear.x = random.uniform(-1.0, 1.0)
            msg.current_velocity.linear.y = random.uniform(-0.5, 0.5)
            msg.current_velocity.angular = Vector3()
            msg.current_velocity.angular.z = random.uniform(-0.5, 0.5)
        
        # 오류 정보
        error_count = random.randint(0, 3)
        msg.error_flags = [False] * 5  # 5개의 시스템 오류 플래그
        msg.error_messages = []
        
        if error_count > 0:
            for i in range(error_count):
                if i < len(msg.error_flags):
                    msg.error_flags[i] = True
                    msg.error_messages.append(f"시스템 {i+1} 경고")
        
        # 메시지 발행
        self.publisher_.publish(msg)
        
        status_names = {
            RobotStatus.STATUS_IDLE: "대기중",
            RobotStatus.STATUS_MOVING: "이동중", 
            RobotStatus.STATUS_ERROR: "오류",
            RobotStatus.STATUS_CHARGING: "충전중"
        }
        
        self.get_logger().info(
            f'로봇 상태 발행: {status_names[msg.status]}, '
            f'배터리={msg.battery_level:.1f}%, '
            f'센서수={len(msg.sensor_readings)}, '
            f'위치=({msg.current_pose.position.x:.2f}, {msg.current_pose.position.y:.2f})'
        )
    
    def update_robot_state(self):
        """로봇 상태를 시뮬레이션하는 메서드"""
        self.status_counter += 1
        
        # 배터리 소모 시뮬레이션
        if self.current_status == RobotStatus.STATUS_MOVING:
            self.battery_level -= random.uniform(0.5, 1.5)
            # 위치 업데이트
            self.position_x += random.uniform(-0.5, 0.5)
            self.position_y += random.uniform(-0.5, 0.5)
        elif self.current_status == RobotStatus.STATUS_CHARGING:
            self.battery_level += random.uniform(2.0, 5.0)
        else:
            self.battery_level -= random.uniform(0.1, 0.3)
        
        # 배터리 레벨 제한
        self.battery_level = max(0.0, min(100.0, self.battery_level))
        
        # 상태 전환 로직
        if self.battery_level < 20.0 and self.current_status != RobotStatus.STATUS_CHARGING:
            self.current_status = RobotStatus.STATUS_CHARGING
        elif self.battery_level > 80.0 and self.current_status == RobotStatus.STATUS_CHARGING:
            self.current_status = RobotStatus.STATUS_IDLE
        elif self.current_status == RobotStatus.STATUS_IDLE and self.status_counter % 5 == 0:
            self.current_status = RobotStatus.STATUS_MOVING
        elif self.current_status == RobotStatus.STATUS_MOVING and self.status_counter % 10 == 0:
            self.current_status = RobotStatus.STATUS_IDLE

def main(args=None):
    rclpy.init(args=args)
    
    robot_status_publisher = RobotStatusPublisher()
    
    try:
        rclpy.spin(robot_status_publisher)
    except KeyboardInterrupt:
        pass
    
    robot_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()