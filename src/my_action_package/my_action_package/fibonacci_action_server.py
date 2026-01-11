#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci
import time


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci Action Server 시작됨')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Goal 수신: {goal_handle.request.order}")

        if goal_handle.request.order < 0:
            goal_handle.abort()
            result = Fibonacci.Result()
            result.sequence = []
            return result

        feedback_msg = Fibonacci.Feedback()
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal 취소됨')
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            sequence.append(sequence[i] + sequence[i - 1])

            feedback_msg.sequence = sequence.copy()
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'피드백 발행: {sequence}')

            time.sleep(1.0)

        # 루프가 끝난 후에 succeed 호출
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f'Goal 성공: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()