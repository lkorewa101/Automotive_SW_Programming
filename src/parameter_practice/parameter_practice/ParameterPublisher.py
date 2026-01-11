#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult


class ParameterPublisher(Node):
    def __init__(self):
        super().__init__('parameter_publisher')
        
        # 매개변수 선언
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('message_content', 'Hello from Parameter Node')
        self.declare_parameter('node_active', True)
        
        # 계층적 매개변수 선언 예시
        self.declare_parameters(
            namespace='',
            parameters=[
                ('advanced_settings.debug_mode', False),
                ('advanced_settings.log_level', 'info'),
                ('advanced_settings.buffer_size', 1000)
            ]
        )
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(String, 'parameter_topic', 10)
        
        # 초기 주기로 타이머 생성
        frequency = self.get_parameter('publish_frequency').value
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)
        
        # 매개변수 콜백 등록 (실습 3)
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Parameter Publisher Node Started')
        self.get_logger().info(f'Initial frequency: {frequency} Hz')

    def timer_callback(self):
        # 매개변수 값 가져오기
        active = self.get_parameter('node_active').value
        
        if not active:
            return
        
        message_content = self.get_parameter('message_content').value
        
        msg = String()
        msg.data = message_content
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def parameter_callback(self, params):
        """매개변수 변경 시 호출되는 콜백 함수 (실습 3)"""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')
            
            # publish_frequency 검증
            if param.name == 'publish_frequency':
                if param.value <= 0:
                    result.successful = False
                    result.reason = f'Invalid frequency: {param.value}. Must be positive.'
                    return result
                
                # 타이머 주기 업데이트
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / param.value, self.timer_callback)
                self.get_logger().info(f'Timer period updated to {1.0/param.value:.2f} seconds')
            
            # message_content 검증
            elif param.name == 'message_content':
                if len(param.value) == 0:
                    result.successful = False
                    result.reason = 'Message content cannot be empty.'
                    return result
            
            # node_active 검증
            elif param.name == 'node_active':
                if param.value:
                    self.get_logger().info('Node activated')
                else:
                    self.get_logger().info('Node deactivated')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ParameterPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()