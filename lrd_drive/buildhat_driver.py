import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from buildhat import Motor, MotorPair

from diff_drive_kinematics import i_kinematics, angular_to_linear

class RobotNode(Node):
    def __init__(self, left_motor: str = 'A', right_motor: str = 'B', max_speed: int = 100):
        print('starting...')
        super().__init__('robot_node')
        self.max_speed = max_speed
        self.motors = MotorPair(left_motor, right_motor)
        
        self.motors.release = False  # Set motor to not release after running
        self.motors.set_default_speed(self.max_speed)

        self.create_subscription(
            Twist, 
            '/diff_cont/cmd_vel_unstamped', 
            self.run_motors, 
            10)
        
    def run_motors(self, twist_msg):
        phi_l, phi_r = i_kinematics(twist_msg.linear.x * self.max_speed, 
                                    twist_msg.angular.z * self.max_speed)
        speedl, speedr = -angular_to_linear(phi_l), angular_to_linear(phi_r)
        
        # make sure we aren't going over the limit
        speedl = math.copysign(min(abs(speedl), self.max_speed), speedl) 
        speedr = math.copysign(min(abs(speedr), self.max_speed), speedr) 

        self.motors.start(speedl, speedr)

        print(f'Driving with: l={speedl}, r={speedr}')
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()