from definitions import *
from gyrofunctions import *
from linetracking import *

'''
Ω my god
Gyro_Reset(0)
gyro_turn(target=90, power=TURN, mode=0, stop=0)
await.wait(0)
drive_base.stop()
drive_base.straight(distance=50)
line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=1000)
line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=KD)
line_track_junction_left(minspeed=JUNCTION, kp=KP, kd=KD)
line_track_junction_right(minspeed=JUNCTION, kp=KP, kd=KD)
'''


FAST = 90
TURN = 80
JUNCTION = 80
KP = 0.2
KD = 8
colors = [Color.RED, Color.YELLOW, Color.BLUE, Color.GREEN]
colour_list = []

def put_block(junctions: int):
    for _ in range(junctions):
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=210)
        line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=KD)

    drive_base.straight(distance=50)
    gyro_turn(target=-90, power=TURN, mode=0, stop=0)
    line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=100)


def get_block(colour: int):
    line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)
    line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=KD)

    if colour == 0:  # red left
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)
        line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=KD)
        drive_base.straight(distance=50)
        grabby_thingie.dc(67)
        gyro_turn(target=-90, power=TURN, mode=0, stop=0)
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)

    elif colour == 1:  # yellow right
        drive_base.straight(distance=50)
        gyro_turn(target=90, power=TURN, mode=0, stop=0)
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)
        gyro_turn(target=-180, power=TURN, mode=0, stop=0)

    elif colour == 2:  # green left
        drive_base.straight(distance=50)
        gyro_turn(target=-90, power=TURN, mode=0, stop=0)
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)
        gyro_turn(target=-180, power=TURN, mode=0, stop=0)

    elif colour == 3:  # blue right
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)
        line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=10)
        drive_base.straight(distance=50)
        gyro_turn(target=90, power=TURN, mode=0, stop=0)
        line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=200)
        gyro_turn(target=-180, power=TURN, mode=0, stop=0)

    gyro_turn(target=-180, power=TURN, mode=0, stop=0)
    line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=KD)

    drive_base.straight(distance=50)

    if colour in (0, 2):
        gyro_turn(target=90, power=TURN, mode=0, stop=0)
    else:
        gyro_turn(target=-90, power=TURN, mode=0, stop=0)

Gyro_Reset(0)
drive_base.straight(distance=310)
gyro_turn(target=-90, power=TURN, mode=0, stop=0)
line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=50)
line_track_junction_both(minspeed=JUNCTION, kp=KP, kd=KD)
drive_base.straight(distance=50)
gyro_turn(target=-90, power=TURN, mode=0, stop=0)
line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=50)
line_track_junction_right(minspeed=JUNCTION, kp=KP, kd=KD)
drive_base.straight(distance=50)
gyro_turn(target=90, power=TURN, mode=0, stop=0)

for i in range(4): #colour detection
  while not(Object_Color_Detection.color(surface=True) in colors):
    left_motor.dc(50)
    right_motor.dc(50)
  if Object_Color_Detection.color(surface=True) == Color.RED:
    colour_list.append(0)
  elif Object_Color_Detection.color(surface=True) == Color.YELLOW:
    colour_list.append(1)
  elif Object_Color_Detection.color(surface=True) == Color.GREEN:
    colour_list.append(2)
  elif Object_Color_Detection.color(surface=True) == Color.BLUE:
    colour_list.append(3)
  print(Object_Color_Detection.color(surface=True))
  line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=100)
gyro_turn(target=180, power=TURN, mode=0, stop=0)
line_track_distance(maxspeed=FAST, kp=KP, kd=KD, distance=300)
line_track_junction_left(minspeed=JUNCTION, kp=KP, kd=KD)
drive_base.straight(distance=50)
gyro_turn(target=-90, power=TURN, mode=0, stop=0)
line_track_junction_left(minspeed=JUNCTION, kp=KP, kd=KD)
get_block(colour=colour_list[0])
line_track_junction_right(minspeed=JUNCTION, kp=KP, kd=KD)
gyro_turn(target=180, power=TURN, mode=0, stop=0)
