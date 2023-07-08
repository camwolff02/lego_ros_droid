from buildhat import Motor

def main():
    right_motor = Motor('A')
    left_motor = Motor('B')

    right_motor.run_for_seconds(5, speed=50)
    left_motor.run_for_seconds(5, speed=50)

if __name__ == "__main__":
    main()