"""
tutorials used:
odometry calculation: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom/
quaternion calculation: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from diff_drive_kinematics import diff_drive_fk


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class DiffDriveOdometry(Node):

    def __init__(self):
        super().__init__('diff_drive_odom')  # type: ignore
        print('starting...')

        # Declare robot specific odometry parameters
        self.child_frame_id = self.declare_parameter(
          'child_frame_id', 'base_link').get_parameter_value().string_value

        self.radius = self.declare_parameter(
            'wheel_radius', 0.044).get_parameter_value().double_value

        self.separation = self.declare_parameter(
            'wheel_separation', 0.12).get_parameter_value().double_value

        # declare members for computing movement over time
        self.x, self.y, self.theta = [0.0]*3
        self.vx, self.vy, self.vtheta = [0.0]*3

        self.last_time = self.get_clock().now().to_msg()

        # Initialize the odometry publisher and transform broadcaster
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # start listening for wheel velocities
        self.create_subscription(
            Float64MultiArray,
            '/wheel_vels',
            self.compute_robot_vels,
            50)

        print('ready!')

    def compute_robot_vels(self, msg: Float64MultiArray) -> None:
        wheel_vels = msg.data.tolist()

        if len(wheel_vels) < 2 or self.radius <= 0.0 or self.separation <= 0.0:
            if len(wheel_vels) < 2: 
                self.get_logger().warning(
                    '"/wheel_vels" must be in format [left_wheel_vel, right_wheel_vel]')
            if self.radius <= 0:
                self.get_logger().warning(
                    '"wheel_radius" not set, odometry not calculated', once=True)
            if self.separation <= 0:
                self.get_logger().warning(
                    '"wheel_separation" not set, odometry not calculated', once=True)
        else:
            left_wheel_vel, right_wheel_vel = wheel_vels[0]/100, wheel_vels[1]/100
            self.vx, self.vtheta = diff_drive_fk(
                left_wheel_vel, right_wheel_vel, self.radius, self.separation)

        # now that we have velocities, compute odometry
        self.compute_odom()


    def compute_odom(self) -> None:
        # calculate time between odometry calculations
        current_time = self.get_clock().now().to_msg()
        dt = (current_time.sec + current_time.nanosec*10**-9) \
            - (self.last_time.sec + self.last_time.nanosec*10**-9)
        self.last_time = current_time
        print(dt)

        # do nothing if the radius or separation is not set
        if self.radius <= 0.0 or self.separation <= 0.0: 
            if self.radius <= 0:
                self.get_logger().warning(
                    '"wheel_radius" not set, odometry not calculated', once=True)
            if self.separation <= 0:
                self.get_logger().warning(
                    '"wheel_separation" not set, odometry not calculated', once=True)
        else:
            # compute odometry in a typical way given the velocities of the robot
            delta_x = (self.vx*math.cos(self.theta) - self.vy*math.sin(self.theta)*dt)
            delta_y = (self.vx*math.sin(self.theta) + self.vy*math.cos(self.theta)*dt)
            delta_theta = self.vtheta * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # since all odometry is 6DOF we'll need a quaternion
            q = quaternion_from_euler(0, 0, self.theta)
            odom_quat = Quaternion()
            odom_quat.x = q[0]
            odom_quat.y = q[1]
            odom_quat.z = q[2]
            odom_quat.w = q[3]

            # first, we'll publish the transform over tf
            odom_trans = TransformStamped()
            odom_trans.header.stamp = current_time
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = self.child_frame_id

            odom_trans.transform.translation.x = self.x
            odom_trans.transform.translation.y = self.y
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quat

            # send the transform 
            self.tf_broadcaster.sendTransform(odom_trans)

            # next, we'll publish the odometry message over ROS2
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = 'odom'

            # set the position
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.orientation = odom_quat

            # set the velocity
            odom.child_frame_id = self.child_frame_id
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.vtheta

            # publish the message
            self.odom_pub.publish(odom)
            self.get_logger().info('starting odometry publishing...', once=True)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == '__main__':
    main()
