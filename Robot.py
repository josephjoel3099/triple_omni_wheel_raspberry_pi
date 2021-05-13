# Importing main libraries
import numpy as np
import RPi.GPIO as GPIO
import curses
import math
import csv
import matplotlib.pyplot as plt

# Initializing global variables
pi = np.pi

# Initializing variables and function for printing
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

GPIO.setmode(GPIO.BOARD)  # initializing the Raspberry Pi GPIO pin nomenclature
GPIO.setwarnings(False)  # Disables unwanted warnings


# Custom library containing all motor params and functions
class Motor:
    def __init__(self, name, in1, in2, pwm_pin, EncA, EncB, CPR, wheel_dia):

        # All motor params
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm_pin
        self.EncA = EncA
        self.EncB = EncB
        self.count = 0
        self.rev = 0
        self.dist_data = 0
        self.count_data = [0]
        self.unit_vel = [0]
        self.vel = 0
        self.total = 0
        self.CPR = CPR
        self.wheel_dia = wheel_dia
        self.pwm_check = 0
        self.sign_check = 0

    def Motor_setup(self):  # Initializes motor pins on the Raspberry Pi
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.EncA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.EncB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

    def Motor_pwm(self):  # Initialized pwm pin
        self.m_pwm = GPIO.PWM(self.pwm_pin, 100)

    def edge_detect(self):  # Initializes events to help get feedback through encoders
        GPIO.add_event_detect(self.EncA, GPIO.BOTH, self.rotation_decode)
        GPIO.add_event_detect(self.EncB, GPIO.BOTH, self.rotation_decode)

    def count_reset(self):  # Count reset
        self.count = 0

    def rotation_decode(self, tag):  # X4 decoding
        if tag == self.EncA and self.pwm_check != 0:
            if GPIO.input(self.EncA) == GPIO.HIGH:
                if GPIO.input(self.EncB) == GPIO.LOW and self.sign_check == 1:
                    self.count += 1
                elif GPIO.input(self.EncB) == GPIO.HIGH and self.sign_check == 0:
                    self.count -= 1

            elif GPIO.input(self.EncA) == GPIO.LOW:
                if GPIO.input(self.EncB) == GPIO.HIGH and self.sign_check == 1:
                    self.count += 1
                elif GPIO.input(self.EncB) == GPIO.LOW and self.sign_check == 0:
                    self.count -= 1

        elif tag == self.EncB and self.pwm_check != 0:
            if GPIO.input(self.EncB) == GPIO.HIGH:
                if GPIO.input(self.EncA) == GPIO.HIGH and self.sign_check == 1:
                    self.count += 1
                elif GPIO.input(self.EncA) == GPIO.LOW and self.sign_check == 0:
                    self.count -= 1

            elif GPIO.input(self.EncB) == GPIO.LOW:
                if GPIO.input(self.EncA) == GPIO.LOW and self.sign_check == 1:
                    self.count += 1
                elif GPIO.input(self.EncA) == GPIO.HIGH and self.sign_check == 0:
                    self.count -= 1

    def Motor_run(self):  # Converting encoder pulses to real world params
        self.dist_data = self.count * np.pi * self.wheel_dia / self.CPR
        self.rev = self.count / self.CPR
        self.count_data.append(self.count)

    def pwm(self, duty, sign):  # Runs motor in the required pwm and direction
        self.pwm_check = duty
        self.sign_check = sign
        self.m_pwm.start(0)
        if sign == 1:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            self.m_pwm.ChangeDutyCycle(duty)
        else:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            self.m_pwm.ChangeDutyCycle(duty)

    def STOP_pwm(self):  # Stops motor
        self.m_pwm.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def Vel(self, t, interval):  # Calculates velocity of the motor
        if t != 0:
            self.unit_vel.append(2 * pi * (self.count_data[-1] - self.count_data[-2]) / (self.CPR * interval * 4))

            if self.unit_vel[-1] != 0:
                self.vel = sum(self.unit_vel) / len(self.unit_vel)
            else:
                self.vel = 0

    def data_print(self, t, motor_number):  # prints data
        stdscr.addstr(motor_number + 1, 0, 'Runtime: %0.1f || %s: Count: %d | Rev: %d | Dist: %0.2f | Vel: %0.2f\t' % (
            t, self.name, self.count, self.rev, self.dist_data, self.vel))
        stdscr.refresh()

    def stop_print(self):  # stops printing
        curses.echo()
        curses.nocbreak()
        curses.endwin()


# Custom library containing all robot kinematics params and functions
class Kinematics:
    def __init__(self, r, d):

        # All robot kinematic params
        self.r = r
        self.d = d
        self.w1 = 0
        self.w2 = 0
        self.w3 = 0
        self.pwm1 = 0
        self.pwm2 = 0
        self.pwm3 = 0
        self.Vx = 0
        self.Vy = 0
        self.Wz = 0

    def ikine(self, iVx, iVy, iWz):  # Inverse kinematics
        h_mat = [[-self.d / self.r, 1 / self.r, 0],
                 [-self.d / self.r, -1 / (2 * self.r), -(np.sin(np.pi / 3) / self.r)],
                 [-self.d / self.r, -1 / (2 * self.r), (np.sin(np.pi / 3) / self.r)]
                 ]
        robot_vel = [[iWz], [iVx], [iVy]]

        [[self.w1], [self.w2], [self.w3]] = np.dot(h_mat, robot_vel)

    def wheel_pwm(self):  # PWM publisher

        # Convert wheel velocity in rad/s to pwm in %
        self.pwm1 = np.round(self.w1, 2) / 14.75 * 100
        self.pwm2 = np.round(self.w2, 2) / 14.25 * 100
        self.pwm3 = np.round(self.w3, 2) / 14.25 * 100

        if self.pwm1 < 0:
            sign1 = 0
        else:
            sign1 = 1

        if self.pwm2 < 0:
            sign2 = 0
        else:
            sign2 = 1

        if self.pwm3 < 0:
            sign3 = 0
        else:
            sign3 = 1

        return abs(self.pwm1), sign1, abs(self.pwm2), sign2, abs(self.pwm3), sign3

    def fkine(self, v1, v2, v3):  # Forward kinematics
        h_mat = [[-self.d / self.r, 1 / self.r, 0],
                 [-self.d / self.r, -1 / (2 * self.r), -(np.sin(np.pi / 3) / self.r)],
                 [-self.d / self.r, -1 / (2 * self.r), (np.sin(np.pi / 3) / self.r)]
                 ]
        h_mat = np.linalg.inv(h_mat)
        wheel_vel = [[v1], [v2], [v3]]

        [[self.Wz], [self.Vx], [self.Vy]] = np.dot(h_mat, wheel_vel)

    def kine_print(self):  # Prints data
        stdscr.addstr(0, 0, '\t\t       * * * BETA * * *')
        stdscr.addstr(6, 0, '                w1: %0.2f | w2: %0.2f | w3: %0.2f\t\t' % (self.w1, self.w2, self.w3))
        stdscr.addstr(7, 0,
                      '           pwm1: %0.3f | pwm2: %0.3f | pwm3: %0.3f\t\t' % (self.pwm1, self.pwm2, self.pwm3))
        stdscr.addstr(8, 0, '\t      Vx: %0.3f | Vy: %0.3f | Wz: %0.3f\t\t' % (self.Vx, self.Vy, self.Wz))
        stdscr.refresh()

    def end_print(self):  # Prints data
        stdscr.addstr(12, 0, '\t\t       Press p to exit\t')
        stdscr.refresh()

    def wait(self):  # Prints data
        stdscr.addstr(12, 0, '\t\t        Please Wait!\t')
        stdscr.refresh()

    def qr_scan(self):  # Prints data
        stdscr.addstr(3, 0, '\t              Please scan QR code')
        stdscr.refresh()

    def robot_stop(self):  # stops printing
        curses.echo()
        curses.nocbreak()
        curses.endwin()


# Custom library containing all robot trajectory params and functions
class Trajectory:
    def __init__(self, start_position, goal_position, sample_time, robot_max_vel):

        # All trajectory params
        self.start_position = start_position
        self.goal_position = goal_position
        self.sample_time = sample_time
        self.var_x = []
        self.var_y = []
        self.dist = 0
        self.robot_max_vel = robot_max_vel
        self.max_time = 0
        self.tarr = []
        self.x_dot = 0
        self.y_dot = 0
        self.r_vel_x = []
        self.r_vel_y = []
        self.theta = 0
        self.robot_pose = []
        self.robot_vel = []
        self.x = 0
        self.y = 0
        self.intercept = 0

    def elu_dist(self):  # Calculates enclidean distance between start and goal points
        self.dist = (np.sqrt(
            (np.square(self.goal_position[0] - self.start_position[0]) + np.square(
                self.goal_position[1] - self.start_position[1]))))
        self.max_time = self.dist / self.robot_max_vel
        self.tarr = np.arange(0, self.max_time, self.sample_time)

    def planner(self):  # Plans a straight line path and a ramp-up steady ramp-down trajectory
        self.theta = math.atan2((self.goal_position[1] - self.start_position[1]),
                                (self.goal_position[0] - self.start_position[0]))
        self.x_dot = self.robot_max_vel * np.cos(self.theta)
        self.y_dot = self.robot_max_vel * np.sin(self.theta)

        try:
            self.r_vel_x = np.append(np.linspace(0, self.x_dot, 20),
                                     np.append(np.linspace(self.x_dot, self.x_dot, len(self.tarr) - 40),
                                               np.linspace(self.x_dot, 0, 20)))
            self.r_vel_y = np.append(np.linspace(0, self.y_dot, 20),
                                     np.append(np.linspace(self.y_dot, self.y_dot, len(self.tarr) - 40),
                                               np.linspace(self.y_dot, 0, 20)))
        except ValueError:
            self.r_vel_x = np.linspace(self.x_dot, self.x_dot, len(self.tarr))
            self.r_vel_y = np.linspace(self.y_dot, self.y_dot, len(self.tarr))

        for ax_x, ax_y in zip(self.r_vel_x, self.r_vel_y):
            self.robot_vel.append([ax_x, ax_y, 0])

    def traj_print(self): # pints data
        stdscr.addstr(10, 0, '\t    No. of waypoints: %d | Max time: %0.2f' % (len(self.tarr), self.max_time))
        stdscr.refresh()


# Custom library containing all robot measurement params and functions
class Measure:
    def __init__(self):
        # All measured params
        self.time = []
        self.x_in_vel = []
        self.y_in_vel = []
        self.x_out_vel = []
        self.y_out_vel = []
        self.A = []
        self.B = []
        self.C = []
        self.in_VelA = []
        self.in_VelB = []
        self.in_VelC = []
        self.out_VelA = []
        self.out_VelB = []
        self.out_VelC = []

        self.fields = ['time', 'x_in_vel', 'y_in_vel',
                       'x_out_vel', 'y_out_vel',
                       'EncA', 'EncB', 'EncC',
                       'InVelA', 'InVelB', 'InVelC',
                       'OutVelA', 'OutVelB', 'OutVelC']

        self.filename = "datalog.csv"
        self.rows = []

    # All functions below append data received into an array
    def time_data(self, t):
        self.time.append(t)

    def robot_vel_in_data(self, in_x, in_y):
        self.x_in_vel.append(round(in_x, 2))
        self.y_in_vel.append(round(in_y, 2))

    def robot_vel_out_data(self, out_x, out_y):
        self.x_out_vel.append(round(out_x, 2))
        self.y_out_vel.append(round(out_y, 2))

    def encoder_data(self, a, b, c):
        self.A.append(a)
        self.B.append(b)
        self.C.append(c)

    def motor_vel_in_data(self, in_velA, in_velB, in_velC):
        self.in_VelA.append(round(in_velA, 2))
        self.in_VelB.append(round(in_velB, 2))
        self.in_VelC.append(round(in_velC, 2))

    def motor_vel_out_data(self, out_velA, out_velB, out_velC):
        self.out_VelA.append(round(out_velA, 2))
        self.out_VelB.append(round(out_velB, 2))
        self.out_VelC.append(round(out_velC, 2))

    def data_logger(self):  # Takes all data arrays and arranges it into a spreadsheet
        self.rows = np.transpose(
            [self.time, self.x_in_vel, self.y_in_vel,
             self.x_out_vel, self.y_out_vel,
             self.A, self.B, self.C,
             self.in_VelA, self.in_VelB, self.in_VelC,
             self.out_VelA, self.out_VelB, self.out_VelC])

        with open(self.filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(self.fields)
            csvwriter.writerows(self.rows)

    def save_plots(self):  # Plots the stored data and saves them into as images
        plt.plot(self.time, self.x_in_vel, label="Robot In Vel")
        plt.plot(self.time, self.x_out_vel, label="Robot Out Vel")
        plt.xlabel('Time (s)')
        plt.ylabel('Robot Velocity (m/s)')
        plt.title('Robot Velocity in X-axis')
        plt.legend()
        plt.savefig('Robot Velocity in X-axis.png', dpi=300, bbox_inches='tight')

        plt.figure()
        plt.plot(self.time, self.y_in_vel, label="Robot In Vel")
        plt.plot(self.time, self.y_out_vel, label="Robot Out Vel")
        plt.xlabel('Time (s)')
        plt.ylabel('Robot Velocity (m/s)')
        plt.title('Robot Velocity in Y-axis')
        plt.legend()
        plt.savefig('Robot Velocity in Y-axis.png', dpi=300, bbox_inches='tight')

        plt.figure()
        plt.plot(self.time, self.A, label="Motor 1 Encoder")
        plt.plot(self.time, self.B, label="Motor 2 Encoder")
        plt.plot(self.time, self.C, label="Motor 3 Encoder")
        plt.xlabel('Time (s)')
        plt.ylabel('Encoder Counts (No.)')
        plt.title('Encoder Counts')
        plt.legend()
        plt.savefig('Encoder Counts.png', dpi=300, bbox_inches='tight')

        plt.figure()
        plt.plot(self.time, self.in_VelA, label="Motor 1 In Vel")
        plt.plot(self.time, self.out_VelA, label="Motor 1 Out Vel")
        plt.xlabel('Time (s)')
        plt.ylabel('Motor 1 Vel (rad/s)')
        plt.title('Motor 1 Velocity')
        plt.legend()
        plt.savefig('Motor 1 Velocity.png', dpi=300, bbox_inches='tight')

        plt.figure()
        plt.plot(self.time, self.in_VelB, label="Motor 2 In Vel")
        plt.plot(self.time, self.out_VelB, label="Motor 2 Out Vel")
        plt.xlabel('Time (s)')
        plt.ylabel('Motor 2 Vel (rad/s)')
        plt.title('Motor 2 Velocity')
        plt.legend()
        plt.savefig('Motor 2 Velocity.png', dpi=300, bbox_inches='tight')

        plt.figure()
        plt.plot(self.time, self.in_VelC, label="Motor 3 In Vel")
        plt.plot(self.time, self.out_VelC, label="Motor 3 Out Vel")
        plt.xlabel('Time (s)')
        plt.ylabel('Motor 3 Vel (rad/s)')
        plt.title('Motor 3 Velocity')
        plt.legend()
        plt.savefig('Motor 3 Velocity.png', dpi=300, bbox_inches='tight')
