import numpy as np
import RPi.GPIO as GPIO
import curses
import math
from functools import lru_cache
from time import sleep

pi = np.pi
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


class Motor:
    def __init__(self, name, in1, in2, pwm_pin, EncA, EncB, CPR, wheel_dia):
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
        self.vel = 0
        self.CPR = CPR
        self.wheel_dia = wheel_dia

    def Motor_setup(self):
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.EncA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.EncB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

    def Motor_pwm(self):
        self.m_pwm = GPIO.PWM(self.pwm_pin, 1000)

    def rotation_decode(self, tag):
        if GPIO.input(self.EncB):
            self.count += 1
        else:
            self.count -= 1

    @lru_cache(maxsize=1000)
    def edge_detect(self):
        GPIO.add_event_detect(self.EncA, GPIO.RISING, callback=self.rotation_decode)

    def Motor_run(self, t, interval):
        self.dist_data = self.count * np.pi * self.wheel_dia / self.CPR
        self.rev = self.count / self.CPR

        if np.mod(t, interval) == 0:
            self.count_data = [self.count_data[-1], self.count]

    def pwm(self, duty, sign):
        if sign == 1:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            self.m_pwm.start(duty)
        else:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            self.m_pwm.start(duty)

    def STOP_pwm(self):
        self.m_pwm.start(0)
        self.m_pwm.stop()
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def Vel(self, interval):
        self.vel = 2 * pi * (self.count_data[-1] - self.count_data[0]) / (self.CPR * interval)

    def data_print(self, t, motor_number):
        stdscr.addstr(motor_number + 1, 0, 'Runtime: %0.1f || %s: Count: %d | Rev: %d | Dist: %0.2f | Vel: %0.2f\t' % (
            t, self.name, self.count, self.rev, self.dist_data, self.vel))


class Kinematics:
    def __init__(self, r, d):
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

    def ikine(self, iVx, iVy, iWz):
        h_mat = [[-self.d / self.r, 1 / self.r, 0],
                 [-self.d / self.r, -1 / (2 * self.r), -(np.sin(np.pi / 3) / self.r)],
                 [-self.d / self.r, -1 / (2 * self.r), (np.sin(np.pi / 3) / self.r)]
                 ]
        robot_vel = [[iWz], [iVx], [iVy]]

        [[self.w1], [self.w2], [self.w3]] = np.dot(h_mat, robot_vel)

    def wheel_pwm(self):
        self.pwm1 = np.round(self.w1, 2) / 7.8 * 100
        self.pwm2 = np.round(self.w2, 2) / 11 * 100
        self.pwm3 = np.round(self.w3, 2) / 11 * 100

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

    def fkine(self, v1, v2, v3):
        h_mat = [[-self.d / self.r, 1 / self.r, 0],
                 [-self.d / self.r, -1 / (2 * self.r), -(np.sin(np.pi / 3) / self.r)],
                 [-self.d / self.r, -1 / (2 * self.r), (np.sin(np.pi / 3) / self.r)]
                 ]
        h_mat = np.linalg.inv(h_mat)
        wheel_vel = [[v1], [v2], [v3]]

        [[self.Wz], [self.Vx], [self.Vy]] = np.dot(h_mat, wheel_vel)

    def kine_print(self):
        stdscr.addstr(0, 0, '\t\t       * * * BETA * * *')
        stdscr.addstr(6, 0, '                w1: %0.2f | w2: %0.2f | w3: %0.2f\t' % (self.w1, self.w2, self.w3))
        stdscr.addstr(7, 0, '           pwm1: %0.3f | pwm2: %0.3f | pwm3: %0.3f\t' % (self.pwm1, self.pwm2, self.pwm3))
        stdscr.addstr(8, 0, '\t      Vx: %0.3f | Vy: %0.3f | Wz: %0.3f\t' % (self.Vx, self.Vy, self.Wz))
        stdscr.refresh()

    def end_print(self):
        stdscr.addstr(10, 0, '\t\t       Press p to exit')
        stdscr.refresh()

    def robot_stop(self):
        curses.echo()
        curses.nocbreak()
        curses.endwin()


class Trajectory:
    def __init__(self, start_position, goal_position, sample_time, robot_max_vel):
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

    def elu_dist(self):
        self.dist = (np.sqrt(
            (np.square(self.goal_position[0] - self.start_position[0]) + np.square(
                self.goal_position[1] - self.start_position[1]))))
        self.max_time = self.dist / self.robot_max_vel
        self.tarr = np.arange(0, self.max_time, self.sample_time)

    def planner(self):
        self.theta = math.atan2((self.goal_position[1] - self.start_position[1]),
                                (self.goal_position[0] - self.start_position[0]))
        self.x_dot = self.robot_max_vel * np.cos(self.theta)
        self.y_dot = self.robot_max_vel * np.sin(self.theta)

        self.r_vel_x = np.append(
            np.append(np.linspace(0, self.x_dot, 20), np.linspace(self.x_dot, self.x_dot, len(self.tarr) - 40)),
            np.linspace(self.x_dot, 0, 20))
        self.r_vel_y = np.append(
            np.append(np.linspace(0, self.y_dot, 20), np.linspace(self.y_dot, self.y_dot, len(self.tarr) - 40)),
            np.linspace(self.y_dot, 0, 20))

        for ax_x, ax_y in zip(self.r_vel_x, self.r_vel_y):
            self.robot_vel.append([ax_x, ax_y, 0])
