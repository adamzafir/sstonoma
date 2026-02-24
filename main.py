from definitions import *
from linetracking import *
from gyrofunctions import *

'''
Gyro_Reset(0)
gyro_turn(180,40,0,0)
wait(0)
drive_base.stop()
decelerate_to_stop(0)
line_track_distance(maxspeed=100,kp=0.19,kd=4.77,distance=2000)
line_track_junction_both(minspeed=20,kp=0.19,kd=4.77)
line_track_junction_left(minspeed=20,kp=0.19,kd=4.77)
line_track_junction_right(minspeed=20,kp=0.19,kd=4.77)
'''

line_track_distance(maxspeed=100,kp=0.19,kd=4.77,distance=200)
line_track_junction_left(minspeed=20,kp=0.19,kd=4.77)
decelerate_to_stop(0.2)

'''
def bw(val):
    return "BLACK" if val < THRESHOLD else "WHITE"

while True:
    L = left_sensor.reflection()
    R = right_sensor.reflection()

    print("Left:", bw(L), "(", L, ")", " |  Right:", bw(R), "(", R, ")")
    wait(200)
'''