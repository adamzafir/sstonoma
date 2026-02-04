from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
grabby_thingie = Motor(Port.F)
left_motor = Motor(Port.A)
right_motor = Motor(Port.B,positive_direction=Direction.COUNTERCLOCKWISE)


drive_base = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=56,
    axle_track=152
)

left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)

def line_track(maxspeed: double, kp: double, kd: double, distance: double):
    perror = 0
    left_motor.reset_angle()
    initialspeed = 12.3
    speed = initialspeed
    while left_motor.angle() < distance:
        error = left_sensor.reflection() - right_sensor.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(speed - (p+d))
        right_motor.dc(speed + (p+d))
        perror = error
        if speed < maxspeed:
            speed += 0.23
            print(speed)
    drive_base.stop()

line_track(maxspeed=100,kp=0.255,kd=6.7,distance=10000)

#testing for junction
print(f"left sensor: {left_sensor.reflection}, right sensor: {right_sensor.reflection}")
