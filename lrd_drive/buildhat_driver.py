import math
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from buildhat import Motor, MotorPair

from diffdrive_kinematics import (
    diffdrive_ik, 
    diffdrive_fk,
    rpm_to_rad,
    rad_to_rpm,
    ang_to_lin,
    lin_to_ang)


class DiffDriveInfo(NamedTuple):
    wheel_rad: float  # [m]
    wheel_sep: float  # [m]
    max_wheel_speed: float  # [rad/s]


class RobotNode(Node):
    def __init__(self, robot_info: DiffDriveInfo, left_motor: str = 'A', right_motor: str = 'B'):
        print('starting...')
        super().__init__('robot_node') # type: ignore

        # setup motors
        self.motors = MotorPair(left_motor, right_motor)
        self.motors.release = False  # Set motor to not release after running
        self.motors.set_default_speed(100)

        # setup math for kinematics and odometry
        self.linear, self.angular = 0.0, 0.0
        self.radius = robot_info.wheel_rad  
        self.separation = robot_info.wheel_sep
        self.max_wheel_speed = robot_info.max_wheel_speed
        self.max_robot_speed, _ = diffdrive_fk(self.max_wheel_speed, self.max_wheel_speed, self.radius, self.separation)
        _, self.max_robot_rotation = diffdrive_fk(-self.max_wheel_speed, self.max_wheel_speed, self.radius, self.separation)

        self.create_subscription(
            Twist, 
            '/diff_cont/cmd_vel_unstamped', 
            self.run_motors, 
            10)
        
        self._wheel_vel_pub = self.create_publisher(Float64MultiArray, '/wheel_vels', 10)
        timer_period = 0.10  # [s]
        self.last_time = self.get_clock().now().to_msg()
        self.create_timer(timer_period, self.publish_wheel_vel)

        self.left_wheel_last_pos = self.motors._leftmotor.get_aposition()
        self.right_wheel_last_pos = self.motors._rightmotor.get_aposition()

        print('ready!')
        
    def publish_wheel_vel(self) -> None:
        # manually calculate timer period for accuracy
        current_time = self.get_clock().now().to_msg()
        dt = (current_time - self.last_time).toSec()
        self.last_time = current_time

        left_wheel_pos = self.motors._leftmotor.get_aposition()
        right_wheel_pos = self.motors._rightmotor.get_aposition()

        left_wheel_vel = (self.left_wheel_last_pos - left_wheel_pos) / dt
        right_wheel_vel = (self.right_wheel_last_pos - right_wheel_pos) / dt

        self.left_wheel_last_pos = left_wheel_vel
        self.right_wheel_last_pos = right_wheel_pos

        wheel_vels = Float64MultiArray()
        wheel_vels.data = [left_wheel_vel, right_wheel_vel]

        self._wheel_vel_pub.publish(wheel_vels)

    def run_motors(self, twist_msg) -> None:
        """TODO redo math so linear and angular correctly map to wheel velocities"""

        # command a fraction of max robot speed based of twist instruction
        linear = twist_msg.linear.x*self.max_robot_speed
        angular = twist_msg.angular.z*self.max_robot_rotation
        left_vel, right_vel = diffdrive_ik(linear, -angular, self.radius, self.separation)
        
        # increase wheel velocity to max
        factor = 100 / 27.49
        left_percent_max_speed = left_vel * factor
        right_percent_max_speed = right_vel * factor

        self.motors.start(-left_percent_max_speed, right_percent_max_speed)
        print(f'Driving with: l={left_percent_max_speed}, r={right_percent_max_speed}')


def main(args=None):
    robot_info = DiffDriveInfo(
        wheel_rad=0.044,  # [m]
        wheel_sep=0.12,  # [m] 
        max_wheel_speed=rpm_to_rad(175.0))  # [rad/s]

    rclpy.init(args=args)
    node = RobotNode(robot_info)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()