import time
from Robot import Motor, Kinematics
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

M1 = Motor("M1", 11, 13, 33, 3, 5, 860, 0.038)
M2 = Motor("M2", 22, 24, 35, 18, 16, 860, 0.038)
M3 = Motor("M3", 31, 29, 37, 8, 10, 850, 0.038)

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
pwm = 100
comp = 2
ch = ['w', 's', 'a', 'd', 'q', 'e', 'f', 'p']

print('Test Started')

for i in range(0, 7):
    char = ch[i]

    if char == "p":
        print("Stop!")
        M1.STOP_pwm()
        M2.STOP_pwm()
        M3.STOP_pwm()
        exit(0)

    if char == "a":
        print("Left pressed")
        M1.pwm(pwm, 0)
        M2.pwm(pwm / comp, 1)
        M3.pwm(pwm / comp, 1)
        time.sleep(button_delay)

    elif char == "d":
        print("Right pressed")
        M1.pwm(pwm, 1)
        M2.pwm(pwm / comp, 0)
        M3.pwm(pwm / comp, 0)
        time.sleep(button_delay)

    elif char == "e":
        print("Right pressed")
        M1.pwm(pwm, 1)
        M2.pwm(pwm, 1)
        M3.pwm(pwm, 1)
        time.sleep(button_delay)

    elif char == "q":
        print("Right pressed")
        M1.pwm(pwm, 0)
        M2.pwm(pwm, 0)
        M3.pwm(pwm, 0)
        time.sleep(button_delay)

    elif char == "w":
        print("Up pressed")
        M1.STOP_pwm()
        M2.pwm(pwm, 0)
        M3.pwm(pwm, 1)
        time.sleep(button_delay)

    elif char == "s":
        print("Down pressed")
        M1.STOP_pwm()
        M2.pwm(pwm, 1)
        M3.pwm(pwm, 0)
        time.sleep(button_delay)

    elif char == "f":
        print("Stop pressed")
        M1.STOP_pwm()
        M2.STOP_pwm()
        M3.STOP_pwm()
        time.sleep(button_delay)

    time.sleep(0.75)
