from definitions import *
from linetracking import *
from gyrofunctions import *

'''
Gyro_Reset(0)
gyro_turn(target=90, power=50, mode=0, stop=0)
wait(0)
drive_base.stop()
drive_base.straight(distance=50, then=Stop.HOLD, wait=True)
line_track_distance(maxspeed=100, kp=0.19, kd=4.77, distance=2000, stop=False)
line_track_junction_both(minspeed=20, kp=0.19,kd=4.77)
line_track_junction_left(minspeed=20, kp=0.19,kd=4.77)
line_track_junction_right(minspeed=20, kp=0.19,kd=4.77)
'''

Gyro_Reset(0)

drive_base.straight(distance=150, then=Stop.HOLD, wait=True)
line_track_junction_both(minspeed=20, kp=0.19,kd=4.77)
drive_base.straight(distance=50, then=Stop.HOLD, wait=True)

gyro_turn(target=-90, power=50, mode=0, stop=0)

line_track_distance(maxspeed=100, kp=0.19, kd=4.77, distance=1000, stop=False)
line_track_junction_both(minspeed=20, kp=0.19, kd=4.77)
drive_base.straight(distance=50, then=Stop.HOLD, wait=True)

gyro_turn(target=-90, power=50, mode=0, stop=0)

line_track_distance(maxspeed=100, kp=0.19, kd=4.77, distance=000, stop=False)
line_track_junction_right(minspeed=20, kp=0.19, kd=4.77)

gyro_turn(target=90, power=50, mode=0, stop=0)

line_track_distance(maxspeed=100, kp=0.19, kd=4.77, distance=000, stop=True)