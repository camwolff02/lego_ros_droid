#TODO Cam needs to finish
import timeit

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from tag_interface.action import DriveToGps


class DriveToGpsActionServer(Node):

    def __init__(self):
        super().__init__('drive_to_gps_action_server')

        self._action_server = ActionServer(
            self,
            DriveToGps,
            'drive_to_gps',
            self._execute_callback)
        
    def _execute_callback(self, goal_handle):
        """Called when someone sends a goal to the action server."""
        try:
            x, y = goal_handle.gps_waypoint.latitude, goal_handle.gps_waypoint.longitude
            if x is None or y is None:
                self.get_logger().info('MISSING input 😢')
                return
        except:
            self.get_logger().info('BAD input 😢')
            return
        
        self.get_logger().info('Executing goal... 🤔')

        feedback_vel_msg = DriveToGps.Feedback()
        feedback_vel_msg.cmd_vel = [1.0, 1.0]

        # send velocity messages for 5 seconds
        t_0 = timeit.default_timer()
        while timeit.default_timer() - t_0 < 5:
            goal_handle.publish_feedback(feedback_vel_msg)

        goal_handle.succeed()  # report goal state to client

        # define result message
        result = DriveToGps.Result()
        result.succeeded =  bool(input('Did we succeed? 👀 (true/false): '))
        return result

