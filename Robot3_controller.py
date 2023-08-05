# Directions:
# -------------------
# Python Robot
# 
# Create a Python main function
# Create a constant- board_size, and intitialize it with a tuple (5, 5)
# Create a constant- board_icon, and intialize it to a character/string of your choice. Ex: ‘ - ’ 
# Create a constant- robot_icon, and inialize it to a character/string of your choice. Ex: ‘ :[]: ’
# Create a constant, robot_start_location, and iniƟalize it to a tuple containing a valid index
# in regards to the board_size. Ex: (0,0) 
# Create a local variable within main(), current_robot_location, and iniƟalize it to robot_start_location
# Create a 2D array named board, define a funcƟon, init_board(), that iniƟalizes the 2D array to
# size board_size, and assigns each index to board_icon. Additionally, this function will put
# the robot_icon on the board at robot_start_location. 
# Define a show_board() funcƟon to print out the 2D array as an n x n board.
# Define a show_menu() function to print a menu: 
# 1- Left
# 2- Backwards
# 3- Forwards
# 4- Right
# 5- Exit
#
# Write a while loop that contains the necessary code to show the menu and get user input.
# Write the logic to take user input and move the robot icon one step on the board in relation to what the user 
# chooses in the menu.
# Error check user input: You can use int(input(“Example Prompt”)) inside a try-catch statement
# to check for correct type, and an if-statement to check for menu bounds.
# Use a switch statement or if-else to handle menu logic.
# Use a generalized functioon for robot movement. You should not need to write a funcƟon for
# each movement direction.
# Error check robot movement requests, so as to not move outside the 2D array.
# Note: Make sure to update current_robot_location each Ɵme the robot is moved
#
# ---------------------------------------------------------------------------------------------------------------
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Option(Enum):
    LEFT = 1
    BACKWARD = 2
    FORWARD = 3 
    RIGHT = 4


class GridbotController(Node):
    def __init__(self):
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10) 

        while True:
            try:
                user_choice = Option(int(input("Enter the number of your choice: ")))
            except ValueError:
                print("Invalid input. Please enter a number 0 to 4.")

            cmd_vel = Twist()

            match user_choice:
                case Option.FORWARD:
                    cmd_vel.linear.x = 1.0
                case Option.BACKWARD:
                    cmd_vel.linear.x = -1.0
                case Option.LEFT:
                    cmd_vel.linear.y = -1.0
                case Option.RIGHT:
                    cmd_vel.linear.y = 1.0

            self._publisher.publish(cmd_vel)

    def show_menu(self):
        print("Menu:")
        print("[1] to move left")
        print("[2] to move backward")
        print("[3] to move forward")
        print("[4] to move right")
        print("[ctrl+c] to quit")


def main():
    rclpy.init()
    node = GridbotController()
    rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  
    main()
