# Directions:
# -------------------
# Python Robot
# 
# Create a Python main function
# Create a constant- board_size, and intitialize it with a tuple (5, 5)
# Create a constant- board_icon, and intialize it to a character/string of your choice. Ex: ‘ - ’ 
# Create a constant- robot_icon, and iniƟalize it to a character/string of your choice. Ex: ‘ :[]: ’
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
# Note: Make sure to update current_robot_locaƟon each Ɵme the robot is moved
#
# ---------------------------------------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# hyperparameters
board_size = (10, 10)
board_icon = '⬛'
robot_icon = '🤖'
robot_start_location = (0, 0)


class GridbotViewer(Node):
    def __init__(self):
        self.current_robot_location = robot_start_location
        self.board = self.init_board()
        
        self.show_board()
        self.create_subscription(Twist, 'cmd_vel', self.update_board, 10)

    def init_board(self):
        rows, cols = board_size
        board = [[board_icon for _ in range(cols)] for _ in range(rows)]
        row, col = robot_start_location
        board[row][col] = robot_icon
        return board

    def update_board(self, cmd_vel):
        self.current_robot_location = self.move_robot(cmd_vel)

    def move_robot(self, cmd_vel):
        row, col = self.current_location
        rows, cols = board_size

        new_row, new_col = self.current_location # This fixed my error. Previouisly this line wasn't in my code
        
        if cmd_vel.linear.x > 0.0:
            new_row = row - 1
        elif cmd_vel.linear.x < 0.0:
            new_row = row + 1
        elif cmd_vel.linear.y < 0.0:
            new_col = col - 1
        elif cmd_vel.linear.y > 0.0:
            new_col = col + 1

        if 0 <= new_row < rows and 0 <= new_col < cols:
            self.board[row][col] = board_icon
            self.board[new_row][new_col] = robot_icon
            self.current_robot_location = (new_row, new_col)
        else:   # Error check
            print("Robot cannot move outside of the board.")


    def show_board(self):  
        for row in self.board:   # ??
            print(' '.join(row))


def main():
    rclpy.init()
    node = GridbotViewer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  
    main()
