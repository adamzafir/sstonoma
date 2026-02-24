from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port, Stop, Button, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

THRESHOLD = 50
speed = 0
hub = PrimeHub()
left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)
Object_Color_Detection = ColorSensor(Port.E)
# Object_Color_Detection.detectable_colors((Color.RED, Color.YELLOW, Color.BLUE, Color.GREEN))
All_Time = StopWatch()
Time_01 = StopWatch()
Gyro_Time_ = StopWatch()
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, Direction.CLOCKWISE)
grabby_thingie = Motor(Port.F, Direction.COUNTERCLOCKWISE)
drive_base = DriveBase(left_motor, right_motor, 62.4, 175)
Program_Overview_left = 'The left side is the core motor control command area; do not make any changes'
Program_Overview_right = 'The right side is the main program; please set the basic parameters first'
Default_Value__Adjustable_ = 'This is just an explanation and can be ignored.'
Black_Line_Value = 20
White_Line_Value = 70
PD_KP = 0.3
PD_KD = 6
Gyro_KP__One_Motor_Movement_ = 7
Gyro_KD__One_Motor_Movement_ = 70
Gyro_KP__Dual_Motor_Movement_ = 4
Gyro_KD__Dual_Motor_Movement_ = 80
Gyro_Move_KP = 6
Gyro_Move_KD = 60
Gyro_Time = 1000
Gray_Value = 60
Default_Value__Not_Adjustable_ = 'This is just an explanation and can be ignored.'
Payload = 0
Payload_at_Distance_01 = 0
Payload_at_Distance_02 = 0
Color_01 = 0
Color_02 = 0
Color_03 = 0
Color_04 = 0
Color_Code = 0
Color_L = 0
Color_R = 0
PD_Error_New = 0
PD_Error_Old = 0
PD_Power = 0
Gyro_Value = 0
Gyro_Relative_Position_ = 0
Gyro_Initial_Position = 0
Gyro_Error_New = 0
Gyro_Error_Old = 0
Gyro_Power = 0
Gyro_Turn_Error = 0
_Move_Power = 0
Move_Delta = 0
Bolt_Feeding_Rotation_Angle = 0

def decelerate_to_stop(seconds):
    deceleration = speed/max(1, seconds*1000)
    while speed > 0:
        left_motor.dc(speed)
        right_motor.dc(speed)
        speed -= deceleration
        wait(0.001)
