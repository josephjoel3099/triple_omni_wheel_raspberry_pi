import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(19, GPIO.IN)
GPIO.setup(21, GPIO.IN)
GPIO.setup(23, GPIO.IN)
GPIO.setup(26, GPIO.IN)
GPIO.setup(32, GPIO.IN)
GPIO.setup(36, GPIO.IN)

#        d6    d1           Y
#                           |
#     d5    IR    d2       -|---- X
#
#        d4    d3


while True:
    d1 = GPIO.input(38)
    d2 = GPIO.input(38)
    d3 = GPIO.input(38)
    d4 = GPIO.input(38)
    d5 = GPIO.input(38)
    d6 = GPIO.input(38)


