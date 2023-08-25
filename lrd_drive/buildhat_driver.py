from typing import NamedTuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from buildhat import MotorPair

from diff_drive_kinematics import (
    diff_drive_ik, 
    diff_drive_fk,
    rpm_to_rad)


class DiffDriveInfo(NamedTuple):
    wheel_rad: float  # [m]
    wheel_sep: float  # [m]
    max_wheel_speed: float  # [rad/s]


class RobotNode(Node):
    """All variables using SI units"""

    def __init__(self, robot_info: DiffDriveInfo, left_motor: str = 'A', right_motor: str = 'B'):
        print('starting...')
        super().__init__('robot_node')  # type: ignore

        # setup motors
        self.motors = MotorPair(left_motor, right_motor)
        self.motors.release = False  # Set motor to not release after running
        self.motors.set_default_speed(100)

        # setup math for kinematics and odometry
        self.radius = robot_info.wheel_rad
        self.separation = robot_info.wheel_sep
        self.max_wheel_speed = robot_info.max_wheel_speed

        fraction = 2/3
        self.max_linear_speed, _ = diff_drive_fk(self.max_wheel_speed*fraction,
                                               self.max_wheel_speed*fraction,
                                               self.radius,
                                               self.separation)
        _, self.max_angular_speed = diff_drive_fk(-self.max_wheel_speed*fraction,
                                                  self.max_wheel_speed*fraction,
                                                  self.radius,
                                                  self.separation)

        self.create_subscription(
            Twist, '/diff_cont/cmd_vel_unstamped', self.run_motors, 10)

        self._wheel_vel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_vels', 10)

        print('ready!')

    def run_motors(self, twist_msg) -> None:
        # transform linear and angular commands into percents of wheel speeds
        linear = twist_msg.linear.x * self.max_linear_speed  
        angular = twist_msg.angular.z * self.max_angular_speed 
        left_wheel_vel, right_wheel_vel = diff_drive_ik(
            linear, -angular, self.radius, self.separation)

        left_percent_max_speed = left_wheel_vel / self.max_wheel_speed * 100
        right_percent_max_speed = right_wheel_vel / self.max_wheel_speed * 100

        # command motors to spin with set percent of max speed
        self.motors.start(-left_percent_max_speed, right_percent_max_speed)
        print(f'Driving with: \
              l={left_percent_max_speed}, \
              r={right_percent_max_speed}')

        # publish wheel velocities for odometry
        wheel_vels = Float64MultiArray()
        wheel_vels.data = [
            self.max_wheel_speed * left_percent_max_speed / 100,  # rad/s
            self.max_wheel_speed * right_percent_max_speed / 100  # rad/s
        ]
        self._wheel_vel_pub.publish(wheel_vels)


def main(args=None):
    robot_info = DiffDriveInfo(
        wheel_rad=0.044,  # [m]
        wheel_sep=0.12,  # [m]
        max_wheel_speed=rpm_to_rad(135.0))  # [rad/s]

    rclpy.init(args=args)
    node = RobotNode(robot_info)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
