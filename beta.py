# Import Libraries
from Robot import Motor, Kinematics, Trajectory, Measure  # Custom libraries made for the system
import RPi.GPIO as GPIO  # Importing raspberry pi libraries
import time  # Other common libraries
import sys, termios, tty
import pyrebase  # Firebase libraries

GPIO.cleanup()  # Resets all Raspberry Pi GPIO pin to zero or ground


# The below function allows the system to interact with the user and take details of the item to be transferred and
# utilizes the custom libraries to plan a path and execute it
def Deploy_Robot():
    # Firebase config allows the code to know where to upload the data
    config = {
        "apiKey": "AIzaSyCZ-Wx5K-Q0F-jad2Q-LxjE2zstLjVslWk",
        "authDomain": "robot-control-c1e61",
        "databaseURL": "https://robot-control-c1e61-default-rtdb.firebaseio.com/Robot%20Control",
        "storageBucket": "robot-control-c1e61.appspot.com"
    }
    firebase = pyrebase.initialize_app(config)  # initializing the firebase variable

    # Function to get user input
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    print('\033c')  # clear terminal

    GPIO.setmode(GPIO.BOARD)  # initializing the Raspberry Pi GPIO pin nomenclature
    GPIO.setwarnings(False)  # Disables unwanted warnings

    # initializing local variables
    interval = 0.01

    # initializing Motors (3 No.s)
    M1 = Motor("M1", 11, 13, 33, 38, 40, 450, 0.038)
    M2 = Motor("M2", 22, 24, 35, 8, 10, 695, 0.038)
    M3 = Motor("M3", 29, 31, 37, 16, 18, 695, 0.038)

    # initializing motor params and functions to help run and retrieve data
    M1.Motor_setup()
    M2.Motor_setup()
    M3.Motor_setup()

    M1.edge_detect()
    M2.edge_detect()
    M3.edge_detect()

    M1.Motor_pwm()
    M2.Motor_pwm()
    M3.Motor_pwm()

    # Stops motors from rotating unnecessarily
    M1.STOP_pwm()
    M2.STOP_pwm()
    M3.STOP_pwm()

    beta = Kinematics(0.019, 0.052)  # initializing robot kinematic params

    # Loop to continuously check for user input
    while True:
        beta.qr_scan()  # printing function
        db = firebase.database()  # initialize firebase database variable
        char = '"Rack 1"'  # db.child("Rack").get().val()  # retrieving data from firebase

        # Checking if data matches
        if char == '"Rack 1"':
            beta_trajectory = Trajectory([0, 0], [0, 1], 0.05, 0.1)  # Location details of Rack 1 along with other
            # robot param

            # custom library functions used for planning the path and trajectory
            beta_trajectory.elu_dist()
            beta_trajectory.planner()
            beta_trajectory.traj_print()

            beta_measure = Measure()  # initializing variable for storing sensor data

            # resetting count to remove any garbage values
            M1.count_reset()
            M2.count_reset()
            M3.count_reset()

            start_time = round(time.time(), 2)  # initializing params to track program time

            while True:

                t = round(time.time(), 2) - start_time  # Calculating elapsed program time

                if t <= beta_trajectory.max_time + 0.5:  # Checking elapsed robot run time

                    # Custom library functions used for converting sensor values to real world values
                    M1.Motor_run()
                    M2.Motor_run()
                    M3.Motor_run()

                    # Code snippet that selects data produced by the trajectory planner and supplies to the robot drive
                    # functions at a particular interval
                    try:
                        i = int(round(t / beta_trajectory.sample_time, 0))  # Indexing

                        # Applying the retrieved values into the robot kinematic function
                        beta.ikine(beta_trajectory.robot_vel[i][0], beta_trajectory.robot_vel[i][1],
                                   beta_trajectory.robot_vel[i][2])

                        # Storing important data retrieved from the sensors onboard the robot
                        beta_measure.time_data(beta_trajectory.tarr[i])
                        beta_measure.robot_vel_in_data(beta_trajectory.robot_vel[i][0], beta_trajectory.robot_vel[i][1])
                        beta_measure.encoder_data(M1.count, M2.count, M3.count)
                        beta_measure.motor_vel_in_data(beta.w1, beta.w2, beta.w3)
                        beta_measure.motor_vel_out_data(M1.vel, M2.vel, M3.vel)
                        beta_measure.robot_vel_out_data(beta.Vx, beta.Vy)
                    except IndexError:  # Added exception to rule out any unexpected errors
                        i = 0

                    [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()  # pwm publisher

                    # running motors according to the values received from the pwm publisher
                    M1.pwm(pwm1, sign1)
                    M2.pwm(pwm2, sign2)
                    M3.pwm(pwm3, sign3)

                    # Velocity calculation
                    M1.Vel(t, interval)
                    M2.Vel(t, interval)
                    M3.Vel(t, interval)

                    # Localization or forward kinematics
                    beta.fkine(M1.vel, M2.vel, M3.vel)

                    # custom printing functions for debugging
                    M1.data_print(t, 1)
                    M2.data_print(t, 2)
                    M3.data_print(t, 3)
                    beta.kine_print()
                    beta_trajectory.traj_print()
                    time.sleep(interval)

                else:  # once the execution of the path is completed the robot stops
                    M1.STOP_pwm()
                    M2.STOP_pwm()
                    M3.STOP_pwm()

                    # Data received is represented as plots for better visualization
                    beta.wait()
                    beta_measure.data_logger()
                    beta_measure.save_plots()
                    beta.end_print()

                    # User confirmation for exiting the routine
                    while True:
                        char = getch()
                        if char == "p":
                            beta.robot_stop()
                            GPIO.cleanup()
                            exit(0)


# The below function allows the user to manually control the robot using a custom-made joystick app
def Teleop():
    config = {
        "apiKey": "AIzaSyCZ-Wx5K-Q0F-jad2Q-LxjE2zstLjVslWk",
        "authDomain": "robot-control-c1e61",
        "databaseURL": "https://robot-control-c1e61-default-rtdb.firebaseio.com/Robot%20Control",
        "storageBucket": "robot-control-c1e61.appspot.com"
    }

    firebase = pyrebase.initialize_app(config)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    M1 = Motor("M1", 11, 13, 33, 38, 40, 860, 0.038)  # IN1 IN2 PWM ENA ENB CPR DIA
    M2 = Motor("M2", 22, 24, 35, 8, 10, 860, 0.038)
    M3 = Motor("M3", 29, 31, 37, 16, 18, 850, 0.038)

    beta = Kinematics(0.038, 0.080)
    beta.robot_stop()

    M1.Motor_setup()
    M2.Motor_setup()
    M3.Motor_setup()

    M1.edge_detect()
    M2.edge_detect()
    M3.edge_detect()

    M1.Motor_pwm()
    M2.Motor_pwm()
    M3.Motor_pwm()

    M1.STOP_pwm()
    M2.STOP_pwm()
    M3.STOP_pwm()

    button_delay = 0.001

    print('Teleop Started')  # Code start conformation

    while True:
        db = firebase.database()
        char = db.child("Direction").get().val()
        print("%s" % char)

        # Code to check the user input and select code to run the robot in the user-selected direction
        if char == 'p':
            print("Stop!")
            M1.STOP_pwm()
            M2.STOP_pwm()
            M3.STOP_pwm()
            exit(0)

        if char == '"a"':
            print("Left pressed")
            beta.ikine(-0.25, 0, 0)
            [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

            M1.pwm(pwm1, sign1)
            M2.pwm(pwm2, sign2)
            M3.pwm(pwm3, sign3)
            time.sleep(button_delay)

        elif char == '"d"':
            print("Right pressed")
            beta.ikine(0.25, 0, 0)
            [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

            M1.pwm(pwm1, sign1)
            M2.pwm(pwm2, sign2)
            M3.pwm(pwm3, sign3)
            time.sleep(button_delay)

        elif char == '"e"':
            print("Right pressed")
            beta.ikine(0, 0, -1.2)
            [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

            M1.pwm(pwm1, sign1)
            M2.pwm(pwm2, sign2)
            M3.pwm(pwm3, sign3)
            time.sleep(button_delay)

        elif char == '"q"':
            print("clockwise rotation")
            beta.ikine(0, 0, 1.2)
            [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

            M1.pwm(pwm1, sign1)
            M2.pwm(pwm2, sign2)
            M3.pwm(pwm3, sign3)
            time.sleep(button_delay)

        elif char == '"w"':
            print("Forward")
            beta.ikine(0, 0.25, 0)
            [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

            M1.pwm(pwm1, sign1)
            M2.pwm(pwm2, sign2)
            M3.pwm(pwm3, sign3)
            time.sleep(button_delay)

        elif char == '"s"':
            print("Down pressed")
            beta.ikine(0, -0.25, 0)
            [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

            M1.pwm(pwm1, sign1)
            M2.pwm(pwm2, sign2)
            M3.pwm(pwm3, sign3)
            time.sleep(button_delay)

        elif char == '"f"':
            print("Stop pressed")
            M1.STOP_pwm()
            M2.STOP_pwm()
            M3.STOP_pwm()
            time.sleep(button_delay)

# End of main functions

# Main function selector
# config = {
#     "apiKey": "AIzaSyCZ-Wx5K-Q0F-jad2Q-LxjE2zstLjVslWk",
#     "authDomain": "robot-control-c1e61",
#     "databaseURL": "https://robot-control-c1e61-default-rtdb.firebaseio.com/Robot%20Control",
#     "storageBucket": "robot-control-c1e61.appspot.com"
# }
#
# firebase = pyrebase.initialize_app(config)
# print('Firebase Started')
# while True:
#
#     db = firebase.database()
#     DR = db.child("Deploy Robots").get().val()
#     MC = db.child("Manual Controller").get().val()
#
#     # below code selects and runs main function that the user has selected
#     if DR == '"Enabled"':
#         print('Deploy Robots Enabled')
Deploy_Robot()
#     elif MC == '"Enabled"':
#         print('Manual Controller Enabled ')
#         Teleop()
