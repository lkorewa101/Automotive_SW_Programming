#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback
        )
        self.get_logger().info('AddTwoInts 서비스 서버가 준비되었습니다.')

    def add_two_ints_callback(self, request, response):
        # 서비스 요청을 처리하는 콜백 함수
        # request: 클라이언트로부터 받은 요청 데이터
        # response: 클라이언트에게 보낼 응답 데이터
        
        # 요청 데이터 로깅
        self.get_logger().info(f'요청 수신: {request.a} + {request.b}')
        
        # 실제 계산 수행
        response.sum = request.a + request.b
        
        # 응답 데이터 로깅
        self.get_logger().info(f'응답 전송: {response.sum}')
        
        return response
    
def main(args=None):
    rclpy.init(args=args)
    
    node = AddTwoIntsServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
