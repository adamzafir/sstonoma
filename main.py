from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

drive_base = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=56,
    axle_track=112
)

left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)

speed = 30
kp = 1
kd = 5
perror = 0
distance = 10000
left_motor.reset_angle()
while left_motor.angle() < distance:
    error = left_sensor.reflection() - right_sensor.reflection()
    p = error * kp
    d = (error - perror) * kd
    left_motor.dc(speed + (p+d))
    right_motor.dc(speed - (p+d))
    perror = error #update previous error
drive_base.stop() 
