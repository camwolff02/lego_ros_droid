#TODO action server needs to be written, use tag_intergace.action.DriveToMarker
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist

from tag_interface.action import DriveToMarker


class DriveToMarkerServer(Node):

    def __init__(self):
        super().__init__('drive_to_marker_server')
        self.action_server = ActionServer(
            self,
            DriveToMarker,
            'drive_to_marker',
            self._drive_to_marker_callback
        )
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    def _drive_to_marker_callback(self, goal_handle):
        self.get_logger().info('Executing action...')

        # Get the requested goal from the action goal
        distance = goal_handle.request.stopping_distance
        is_gateway = goal_handle.request.is_gateway
        # channel to subscribe to for detecting tags
        camera_channel = goal_handle.request.camera_channel 

        # Create a Twist message with desired linear.x velocity
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Adjust the velocity as needed

        # Publish the twist message repeatedly until the desired distance is covered
        covered_distance = 0.0
        while covered_distance < distance:
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Moving rover... Distance covered: {covered_distance:.2f}")
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow other nodes to execute
            covered_distance += abs(twist_msg.linear.x) * 0.1  # Adjust the time interval as needed

        # Stop the rover by publishing a Twist message with zero linear.x velocity
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Action completed. Total distance covered: {covered_distance:.2f}")

        # Set the result and succeed the action
        goal_handle.succeed()

def main(args=None):
    rclpy.init(args=args)

    server = DriveToMarkerServer()

    rclpy.spin(server)

    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()