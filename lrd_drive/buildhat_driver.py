import math
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        self.curr_pos = (0.0, 0, 0.0, 0, 0.0)
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
        
        self._pub = self.create_publisher(Odometry, 'odom', 10)
        self._dt = 0.10  # [s]
        # self.create_timer(self._dt, self.publish_odom)

        print('ready!')
        
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock.now().to_msg()
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 0
        self._pub.publish(msg)




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

        # record wheel velocity for odometry
        left_wheel_vel = left_percent_max_speed * self.max_wheel_speed
        right_wheel_vel = right_percent_max_speed * self.max_wheel_speed

        self.motors.start(-left_percent_max_speed, right_percent_max_speed)
        print(f'Driving with: l={left_percent_max_speed}, r={right_percent_max_speed}')


        # calculate and publish odometry
        self.linear, self.angular = diffdrive_fk(left_wheel_vel, right_wheel_vel, self.radius, self.separation)
        # print(f'Driving with: linear={linear}, angular={angular}')


def main(args=None):
    robot_info = DiffDriveInfo(
        wheel_rad=0.045,  # [m]
        wheel_sep=0.125,  # [m] 
        max_wheel_speed=rpm_to_rad(175.0))  # [rad/s]

    rclpy.init(args=args)
    node = RobotNode(robot_info)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()