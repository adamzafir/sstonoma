from definitions import *
from linetracking import *
from movementtemplate import *

Gyro_Reset(0)
gyro_turn(180,50,0,0)
line_track_distance(maxspeed=100,kp=0.19,kd=4.77,distance=10000)
# line_track_t(maxspeed=100,kp=0.19,kd=4.77)

# def bw(val):
#     return "BLACK" if val < THRESHOLD else "WHITE"

# while True:
#     L = left_sensor.reflection()
#     R = right_sensor.reflection()

#     print("Left:", bw(L), "(", L, ")", " |  Right:", bw(R), "(", R, ")")
#     wait(200)


