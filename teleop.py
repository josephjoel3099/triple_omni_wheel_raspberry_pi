import sys, termios, tty, time
from Robot import Motor, Kinematics
import RPi.GPIO as GPIO
import pyrebase

config = {
  "apiKey": "AIzaSyCZ-Wx5K-Q0F-jad2Q-LxjE2zstLjVslWk",
  "authDomain": "robot-control-c1e61",
  "databaseURL": "https://robot-control-c1e61-default-rtdb.firebaseio.com/Robot%20Control",
  "storageBucket": "robot-control-c1e61.appspot.com"
}

firebase = pyrebase.initialize_app(config)

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

M1 = Motor("M1", 11, 13, 33, 38, 40, 860, 0.038)   #IN1 IN2 PWM ENA ENB CPR DIA
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


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


button_delay = 0.001
pwm = 100
comp = 1

print('Teleop Started')

while True:
    db = firebase.database()
    char = db.child("Direction").get().val()
    print ("%s" % char)

    if char == 'p':
        print("Stop!")
        M1.STOP_pwm()
        M2.STOP_pwm()
        M3.STOP_pwm()
        exit(0)

    if char == '"a"':
        print("Left pressed")
        beta.ikine(-0.1, 0, 0)
        [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

        M1.pwm(pwm1 * comp, sign1)
        M2.pwm(pwm2, sign2)
        M3.pwm(pwm3, sign3)
        time.sleep(button_delay)

    elif char == '"d"':
        print("Right pressed")
        beta.ikine(0.1, 0, 0)
        [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

        M1.pwm(pwm1 * comp, sign1)
        M2.pwm(pwm2, sign2)
        M3.pwm(pwm3, sign3)
        time.sleep(button_delay)

    elif char == '"e"':
        print("Right pressed")
        beta.ikine(0, 0, -0.6)
        [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

        M1.pwm(pwm1 * comp, sign1)
        M2.pwm(pwm2, sign2)
        M3.pwm(pwm3, sign3)
        time.sleep(button_delay)

    elif char == '"q"':
        print("clockwise rotation")
        beta.ikine(0, 0, 0.6)
        [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

        M1.pwm(pwm1 * comp, sign1)
        M2.pwm(pwm2, sign2)
        M3.pwm(pwm3, sign3)
        time.sleep(button_delay)

    elif char == '"w"':
        print("Forward")
        beta.ikine(0, 0.1, 0)
        [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

        M1.pwm(pwm1 * comp, sign1)
        M2.pwm(pwm2, sign2)
        M3.pwm(pwm3, sign3)
        time.sleep(button_delay)

    elif char == '"s"':
        print("Down pressed")
        beta.ikine(0, -0.1, 0)
        [pwm1, sign1, pwm2, sign2, pwm3, sign3] = beta.wheel_pwm()

        M1.pwm(pwm1 * comp, sign1)
        M2.pwm(pwm2, sign2)
        M3.pwm(pwm3, sign3)
        time.sleep(button_delay)

    elif char == '"f"':
        print("Stop pressed")
        M1.STOP_pwm()
        M2.STOP_pwm()
        M3.STOP_pwm()
        time.sleep(button_delay)
