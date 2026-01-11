import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParameterizedPublisher(Node):
    def __init__(self):
        super().__init__('param_publisher')
        
        # 파라미터 선언 (이 두 줄이 반드시 필요)
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('message_prefix', '메시지')
        
        # 파라미터 값 가져오기
        self.frequency = self.get_parameter('publish_frequency').value
        self.prefix = self.get_parameter('message_prefix').value
        
        self.publisher_ = self.create_publisher(String, 'param_topic', 10)
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        self.count = 0
        
        self.get_logger().info(f'주파수: {self.frequency}Hz, 접두사: {self.prefix}')
    
    def timer_callback(self):
        msg = String()
        msg.data = f'{self.prefix} {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'발행: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ParameterizedPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()