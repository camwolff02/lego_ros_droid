import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        """Sends goal to action server, asks to start action.
        
        Args:
            order (int): order of fibonacci sequence to calculate up to
        """
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # send goal to server and define callback if goal is accepted
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when server accepts or rejects goal.

        Args:
            future.result() (bool): true if goal accepted, false otherwise.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected 😤 it's over")
            return

        self.get_logger().info("Goal accepted 🔥 we're back")

        # ask server for result of action and define callback one result is reached
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called when server completes goal.
        
        Args:
            future.result().result: result object holding fibonacci sequence requested 
        """
        result = future.result().result
        self.get_logger().info(f'Result 🤓: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Called when server sends feedback.
        
        Args:
            feedback_msg.feedback: feedback object holding partial fibonacci sequence
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback 👽: {feedback.partial_sequence}');

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    # ask server to complete goal
    action_client.send_goal(10)
    rclpy.spin(action_client)
    print('goal completed!')


if __name__ == '__main__':
    main()