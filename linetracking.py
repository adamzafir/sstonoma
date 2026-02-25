from definitions import *

#line track for distance
def line_track_distance(maxspeed: float, kp: float, kd: float, distance: float, stop: bool):
    global speed
    perror = 0
    left_motor.reset_angle()
    initialspeed = 10
    speed = initialspeed
    while left_motor.angle() < distance:
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed + (p+d))
        right_motor.dc(speed - (p+d))
        perror = error
        if speed < 55:
            speed *= 1.004
            print(speed)
        elif speed < maxspeed:
            speed += 1.15
            print(speed)
    if stop:
        pass
        #drive_base.stop()
        #decelerate_to_stop(0.2)

#line track for T junction
def line_track_junction_both(minspeed: float, kp: float, kd: float):
    global speed
    perror = 0
    left_motor.reset_angle()
    speed = 100
    while not(
        left_sensor.reflection() < THRESHOLD/2 and 
        right_sensor.reflection() < THRESHOLD/2
    ):
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed + (p+d))
        right_motor.dc(speed - (p+d))
        perror = error
        if speed > minspeed:
            speed -= 0.2
            print(speed)

#line track for left junction
def line_track_junction_left(minspeed: float, kp: float, kd: float):
    global speed
    perror = 0
    left_motor.reset_angle()
    speed = 0.9*speed
    while not(
        left_sensor.reflection() < THRESHOLD/2 and 
        right_sensor.reflection() > (2*THRESHOLD)-(THRESHOLD/2)
    ):
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed + (p+d))
        right_motor.dc(speed - (p+d))
        perror = error
        if speed > minspeed:
            speed -= 0.2
            print(speed)

#line track for right junction
def line_track_junction_right(minspeed: float, kp: float, kd: float):
    global speed
    perror = 0
    left_motor.reset_angle()
    speed = 0.9*speed
    while not(
        left_sensor.reflection() > (2*THRESHOLD)-(THRESHOLD/2) and 
        right_sensor.reflection() < THRESHOLD/2
    ):
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed + (p+d))
        right_motor.dc(speed - (p+d))
        perror = error
        if speed > minspeed:
            speed -= 0.2
            print(speed)
        drive_base.stop()