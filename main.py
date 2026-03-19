from definitions import *
from linetracking import *
from gyrofunctions import *

'''
Gyro_Reset(0)
gyro_turn(target=90, power=50, mode=0, stop=0)
wait(0)
drive_base.stop()
line_track_distance(maxspeed=100, kp=0.19, kd=4.77, distance=2000, stop=False)
line_track_junction_both(minspeed=20, kp=0.19,kd=4.77)
line_track_junction_left(minspeed=20, kp=0.19,kd=4.77)
line_track_junction_right(minspeed=20, kp=0.19,kd=4.77)
'''

Gyro_Reset(0)
while True:
    line_track_distance(maxspeed=100, kp=0.19, kd=4.77, distance=1800, stop=False)
    wait(0.5)
    gyro_turn(target=180, power=50, mode=0, stop=0)
    wait(0.5)


'''
def bw(val):
    return "BLACK" if val < THRESHOLD else "WHITE"

while True:
    L = left_sensor.reflection()
    R = right_sensor.reflection()

    print("Left:", bw(L), "(", L, ")", " |  Right:", bw(R), "(", R, ")")
    wait(200)
'''