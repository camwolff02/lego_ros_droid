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
from enum import Enum

board_size = (5, 5)
board_icon = '⬛'
robot_icon = '🤖'
robot_start_location = (0, 0)

class Option(Enum):
    QUIT = 0
    LEFT = 1
    BACKWARD = 2
    FORWARD = 3 
    RIGHT = 4

# Instead of using constants,  I can also be using a dictionary to store the values
# BOARD_INFO = {
#     "board_size" : (5,5),
#     "board_icon" : "-",
#     "robot_icon" : "\/",
#     "robot_start_location" : (3,3)
# }

def init_board():
    rows, cols = board_size
    board = [[board_icon for _ in range(cols)] for _ in range(rows)]
    row, col = robot_start_location
    board[row][col] = robot_icon
    return board


def show_board(board):  # Value of board passed from the while loop in main
    for row in board:   # ??
        print(' '.join(row))


def move_robot(board, current_location, choice):
    row, col = current_location
    rows, cols = board_size

    new_row, new_col = current_location # This fixed my error. Previouisly this line wasn't in my code

    match(choice):
        case Option.FORWARD:
            new_row = row - 1
        case Option.BACKWARD:
            new_row = row + 1
        case Option.LEFT:
            new_col = col - 1
        case Option.RIGHT:
            new_col = col + 1

    if 0 <= new_row < rows and 0 <= new_col < cols:
        board[row][col] = board_icon
        board[new_row][new_col] = robot_icon
        return new_row, new_col
    else:   # Error check
        print("Robot cannot move outside of the board.")
        return current_location


def show_menu():
    
    # The menu
    print("Menu:")
    print("[1] to move left")
    print("[2] to move backward")
    print("[3] to move forward")
    print("[4] to move right")
    print("[0] to quit")


def main():

    current_robot_location = robot_start_location
    board = init_board()

    while True: 
        show_board(board)   # Passes the variable board to show_board()
        show_menu() # Calls the function show_menu and shows the menu's options

        try:
            user_choice = Option(int(input("Enter the number of your choice: ")))
            if user_choice == Option.QUIT:
                print("Ending the program.")
                break
            else:
                current_robot_location = move_robot(board, current_robot_location, user_choice) # Calls function main and passes variables
        except ValueError:
            print("Invalid input. Please enter a number 0 to 4.")

    print("Ending board position:")
    show_board(board)

if __name__ == "__main__":  
    main()
