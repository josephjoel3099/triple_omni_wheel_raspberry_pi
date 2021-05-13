from simple_pid import PID
import time

pid = PID(0.0095, 0, 0, setpoint=1)
vel = 5
out = [10, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

for i in range(0,len(out)):
    control = pid(out[i])
    print('%0.2f' % (control + vel))
    time.sleep(0.5)
