#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci
from action_msgs.msg import GoalStatus


class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )
        self.get_logger().info('Fibonacci Action Client 시작됨')

    def wait_for_server(self):
        self.get_logger().info('액션 서버 대기 중...')
        self._action_client.wait_for_server()
        self.get_logger().info('액션 서버 연결됨')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal 거부됨')
            return

        self.get_logger().info('Goal 수락됨, 결과 대기 중...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'피드백 수신: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'결과 수신 성공: {result.sequence}')
        else:
            self.get_logger().info(f'Goal 실패, 상태코드: {status}')


def main(args=None):
    rclpy.init(args=args)
    client = FibonacciActionClient()

    try:
        order = int(input("피보나치 수열 항 개수: "))
        client.send_goal(order)
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('사용자에 의해 취소됨')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

