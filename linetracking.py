from definitions import *

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
        if speed < 45:
            speed *= 1.004
            print(speed)
        elif speed < maxspeed:
            print("speed MORE then 55")
            speed += 1.2
            print(speed)
    drive_base.stop()

#line track for T junction
def line_track_t(maxspeed: double, kp: double, kd: double):
    perror = 0
    left_motor.reset_angle()
    initialspeed = 10
    speed = initialspeed
    while not(min(left_sensor.reflection(), right_sensor.reflection()) < THRESHOLD/5 and abs(left_sensor.reflection() - right_sensor.reflection())):
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed - (p+d))
        right_motor.dc(speed + (p+d))
        perror = error
        if speed < 45:
            speed *= 1.004
            print(speed)
        elif speed < maxspeed:
            print("speed MORE then 55")
            speed += 1.2
            print(speed)
    drive_base.stop()