import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        # create an action server node with callback function
        # An action has 3 parts:
        #   Fibonacci.Goal - order
        #   Fibonacci.Feedback - partial_sequence
        #   Fibonaci.Result - sequence
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        """Called when someone sends a goal to the action server.
        
        Args:
            goal_handle: how we communicate state (success/failure) 
                         to the client.
        Returns:
            Fibonacci.Result(): result of action (fibonacci sequence).

        Publishes:
            Fibonacci.Feedback() - intermediate steps (sub sequence).
        """
        self.get_logger().info('Executing goal... 🤔')

        # define feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            
            # print debug info and send feedback to client
            self.get_logger().info(f'Feedback 😁: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()  # report goal state to client

        # define result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    """ starts action server """

    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()