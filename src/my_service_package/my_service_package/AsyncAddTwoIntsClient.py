#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import time


class AsyncAddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('async_add_two_ints_client')
        
        # 서비스 클라이언트 생성
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 서비스가용성 확인
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')
            
        self.pending_requests = {} # 내가 보낸 요청 정리
        self.request_id = 0 # 요청 ID 관리 변수

    def send_request_async(self, a, b):
        # 요청 객체 생성 및 데이터 설정
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # 요청 ID 생성
        self.request_id += 1
        current_id = self.request_id
        
        self.get_logger().info(f'비동기 요청 전송 (ID: {current_id}): {a} + {b}')
        
        # 비동기 요청 및 Future 반환
        future = self.cli.call_async(request)
# 요청 정보 저장
        self.pending_requests[current_id] = {
            'future': future,
            'request': (a, b),
            'timestamp': time.time()
        }
        
        # 콜백 함수 등록
        future.add_done_callback(
            lambda fut, req_id=current_id: self.response_callback(fut, req_id)
        )
        
        return current_id
        
    def response_callback(self, future, request_id):
        # 응답 처리 및 정보 삭제
        request_info = self.pending_requests.pop(request_id)
        a, b = request_info['request']
        
        # Future 결과 획득
        try:
            response = future.result()
            self.get_logger().info(f'응답 수신 (ID: {request_id}): {a} + {b} = {response.sum}')
            
        except Exception as e:
            # Exception 처리
            self.get_logger().error(f'서비스 호출 오류 (ID: {request_id}): {e}')

# main 함수는 이미지에 포함되어 있지 않으므로 생략
# 일반적으로 main 함수는 이 클래스의 객체를 생성하고 send_request_async를 호출하는 로직을 포함합니다.
