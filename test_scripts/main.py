import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from buildhat import Motor, MotorPair

from diff_drive_kinematics import i_kinematics, angular_to_linear

class RobotNode(Node):
    def __init__(self, max_speed: int = 100):
        print('starting...')
        super().__init__('robot_node')
        self.max_speed = max_speed
        self.curr_speed = 0.0
        self.motors = MotorPair('A', 'B')
        self.motors.set_default_speed(self.curr_speed)
        # self.motos.set_speed_unit_rpm(True)

        self.create_subscription(
            Twist, 
            '/diffbot_base_controller/cmd_vel_unstamped', 
            self.run_motors, 
            10)
        
    def run_motors(self, msg):
        phi_l, phi_r = i_kinematics(msg.linear.x * self.max_speed, 
                                    msg.angular.z * self.max_speed)
        speedl, speedr = -angular_to_linear(phi_l), angular_to_linear(phi_r)
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