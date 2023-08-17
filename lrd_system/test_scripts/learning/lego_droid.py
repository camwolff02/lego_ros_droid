import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from buildhat import Motor, MotorPair

class RobotNode(Node):
    def __init__(self, max_speed: int = 100):
        print('starting...')
        super().__init__('robot_node')
        self.max_speed = max_speed
        self.motors = MotorPair('A', 'B')

        self.create_subscription(Twist, '/cmd_vel', self.run_motors, 10)
        
    def run_motors(self, cmd_vel):
        factor = 4

        if int(cmd_vel.linear.x) > 0:  # move up
            self.motors.run_for_seconds(1, -self.max_speed, self.max_speed)
        elif int(cmd_vel.linear.x) < 0:  # move down
            self.motors.run_for_seconds(1, self.max_speed, -self.max_speed)
        elif int(cmd_vel.linear.y) > 0:  # turn left
            self.motors.run_for_rotations(0.25, self.max_speed/factor, self.max_speed/factor)
        elif int(cmd_vel.linear.y) < 0:  # turn right
            self.motors.run_for_rotations(0.25, -self.max_speed/factor, -self.max_speed/factor)

        print(f'Driving with: linear x={cmd_vel.linear.x}, angular z={cmd_vel.linear.y}')
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode(max_speed=50)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()