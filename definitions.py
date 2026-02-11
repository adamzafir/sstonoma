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