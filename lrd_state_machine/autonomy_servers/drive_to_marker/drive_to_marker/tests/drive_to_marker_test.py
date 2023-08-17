import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tag_interface.action import DriveToMarker


class DriveToMarkerClient(Node):

    def __init__(self):
        super().__init__('drive_to_marker_client')
        self._action_client = \
            ActionClient(self, DriveToMarker, 'drive_to_marker')
        self._send_goal()

    def _send_goal(self):
        goal_msg = DriveToMarker.Goal()
        goal_msg.stopping_dist = 5.0  
        goal_msg.is_gateway = False
        goal_msg.camera_channel = '/img_raw'

        self._action_client.wait_for_server()
        self.get_logger().info('Sending action goal...')
        future = self._action_client._send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Action goal rejected')
            return

        self.get_logger().info('Action goal accepted')

        self.get_logger().info('Waiting for action to complete...')
        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result:
            self.get_logger().info('Action completed successfully')
        else:
            self.get_logger().info('Action failed')

    def _feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: Distance covered: {feedback_msg.distance_covered:.2f}")


def main(args=None):
    rclpy.init(args=args)

    client = DriveToMarkerClient()

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()