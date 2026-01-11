#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 서비스 서버가 사용 가능할 때까지 대기
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('서비스 서버를 기다리는 중...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.get_logger().info(f'요청 생성: {a} + {b}')

        try:
            # 비동기 호출
            future = self.client.call_async(request)

            # 응답 대기 (동기처럼 사용)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'응답 수신: {response.sum}')
                return response.sum
            else:
                self.get_logger().error('서비스 호출 실패: 응답이 None')
                return None
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    # 1. 노드 객체 생성
    # node = AddTwoIntsClient()
    
    # 2. 커맨드라인 인수 검사 (숫자 2개 필요)
    # 사용법: ros2 run 패키지명 add_two_ints_client <숫자> <숫자>
    if len(sys.argv) != 3:
        print('오류: 정확한 입력 필요합니다.')
        return
        
    # 3. 인수 타입 변환 및 오류 처리
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('오류: 숫자만 입력 가능합니다.')
        return

    # 4. 서비스 요청 및 결과 출력
    client_node = AddTwoIntsClient() # 노드 객체 재사용(또는 재정의)
    result = client_node.send_request(a, b)
    
    if result is not None:
        print(f'결과: {a} + {b} = {result}')

    # 5. 노드 정리 및 종료
    client_node.destroy_node() # client_node 객체 정리
    rclpy.shutdown()


if __name__ == '__main__':
    main()
