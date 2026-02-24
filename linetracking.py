from definitions import *
after_speed = 0
initialspeed = 0
speed = 0
#line track for distance
def line_track_distance(maxspeed: double, kp: double, kd: double, distance: double):
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

#line track for T junction
def line_track_junction_both(minspeed: double, kp: double, kd: double):
    perror = 0
    left_motor.reset_angle()
    initialspeed = 100
    speed = initialspeed
    while not(left_sensor.reflection() < THRESHOLD/3 and right_sensor.reflection() < THRESHOLD/3):
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
def line_track_junction_left(minspeed: double, kp: double, kd: double):
    perror = 0
    left_motor.reset_angle()
    initialspeed = 90
    speed = initialspeed
    while not(left_sensor.reflection() < THRESHOLD/3 and right_sensor.reflection() > (2*THRESHOLD)-(THRESHOLD/3)):
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
def line_track_junction_right(minspeed: double, kp: double, kd: double):
    perror = 0
    left_motor.reset_angle()
    initialspeed = 90
    speed = initialspeed
    while not(left_sensor.reflection() > (2*THRESHOLD)-(THRESHOLD/3)) and right_sensor.reflection() < THRESHOLD/3:
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed + (p+d))
        right_motor.dc(speed - (p+d))
        perror = error
        if speed > minspeed:
            speed -= 0.2
            print(speed)