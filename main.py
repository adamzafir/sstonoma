from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
grabby_thingie = Motor(Port.F)
left_motor = Motor(Port.A)
right_motor = Motor(Port.B,positive_direction=Direction.COUNTERCLOCKWISE)

THRESHOLD = 55

drive_base = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=88,
    axle_track=163
)

left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)

def line_track(maxspeed: double, kp: double, kd: double, distance: double):
    perror = 0
    left_motor.reset_angle()
    initialspeed = 20
    speed = initialspeed
    while left_motor.angle() < distance:
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed - (p+d))
        right_motor.dc(speed + (p+d))
        perror = error
        if speed < 55:
            speed *= 1.0029
            print(speed)
        elif speed < maxspeed:
            print("speed MORE then 55")
            speed += 0.967
            print(speed)
    drive_base.stop()


line_track(maxspeed=100,kp=0.19,kd=4.77,distance=10000)



# def bw(val):
#     return "BLACK" if val < THRESHOLD else "WHITE"

# while True:
#     L = left_sensor.reflection()
#     R = right_sensor.reflection()

#     print("Left:", bw(L), "(", L, ")", " |  Right:", bw(R), "(", R, ")")
#     wait(200)


