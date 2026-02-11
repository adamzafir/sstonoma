from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

# Set up all devices.
prime_hub = PrimeHub()
leftCS = ColorSensor(Port.D)
rightCS = ColorSensor(Port.C)
Object_Color_Detection = ColorSensor(Port.E)
Object_Color_Detection.detectable_colors((Color.RED, Color.YELLOW, Color.BLUE, Color.GREEN))
All_Time = StopWatch()
Time_01 = StopWatch()
Gyro_Time_ = StopWatch()
L_moto = Motor(Port.A, Direction.COUNTERCLOCKWISE)
R_moto = Motor(Port.B, Direction.CLOCKWISE)
C_front = Motor(Port.F, Direction.COUNTERCLOCKWISE)
Car = DriveBase(L_moto, R_moto, 62.4, 175)

# Initialize variables.
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

def Move_mm(adjustment, minSpeed, maxSpeed, accelDist, decelDist, distance, mode, stop):
    global Move_Delta, _Move_Power
    L_moto.reset_angle(0)
    R_moto.reset_angle(0)
    Move_Delta = 0
    while not Move_Delta >= distance:
        Move_Delta = (abs(L_moto.angle()) + abs(R_moto.angle())) * 2721 / 10000
        # setting Acceleration/ Deceleration
        if Move_Delta < accelDist and accelDist != 0:
            _Move_Power = abs(minSpeed) + (abs(maxSpeed) - abs(minSpeed)) * Move_Delta / abs(accelDist)
            if minSpeed < 0 or maxSpeed < 0:
                _Move_Power = -_Move_Power
        elif Move_Delta > distance - decelDist and decelDist != 0:
            _Move_Power = abs(minSpeed) + (abs(maxSpeed) - abs(minSpeed)) * (distance - Move_Delta) / abs(decelDist)
            if minSpeed < 0 or maxSpeed < 0:
                _Move_Power = -_Move_Power
        else:
            _Move_Power = maxSpeed
        L_moto.dc(_Move_Power + adjustment)
        R_moto.dc(_Move_Power - adjustment)
    if mode == 1:
        # detect white
        _Move_Power = minSpeed
        while not leftCS.reflection() >= White_Line_Value or rightCS.reflection() >= White_Line_Value:
            L_moto.dc(_Move_Power + adjustment)
            R_moto.dc(_Move_Power - adjustment)
    if mode == 2:
        # detect black
        _Move_Power = minSpeed
        while not leftCS.reflection() <= Black_Line_Value or rightCS.reflection() <= Black_Line_Value:
            L_moto.dc(_Move_Power + adjustment)
            R_moto.dc(_Move_Power - adjustment)
    if stop == 1:
        L_moto.hold()
        R_moto.hold()
    if stop == 2:
        L_moto.stop()
        R_moto.stop()

def gyro_turn(target, power, mode, stop):
    global Gyro_Turn_Error, Gyro_Relative_Position_, Gyro_Error_New, Gyro_Power, Gyro_Error_Old
    L_moto.reset_angle(0)
    R_moto.reset_angle(0)
    Gyro_Time_.reset()
    Gyro_Turn_Error = 0
    Gyro_Relative_Position_ = Gyro_Relative_Position_ + target
    if mode == 0:
        # spot turn
        Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
        while not (-1 <= Gyro_Error_New <= 1 or Gyro_Time_.time() > Gyro_Time):
            Gyro_Turn_Error = (L_moto.angle() + R_moto.angle()) * 0.5
            Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
            if Gyro_Error_Old != Gyro_Error_New:
                Gyro_Time_.reset()
            Gyro_Power = Gyro_Error_New * Gyro_KP__Dual_Motor_Movement_ + (Gyro_Error_New - Gyro_Error_Old) * Gyro_KD__Dual_Motor_Movement_
            if Gyro_Power >= power:
                Gyro_Power = power
            if Gyro_Power <= -power:
                Gyro_Power = -power
            L_moto.dc(Gyro_Power - Gyro_Turn_Error)
            R_moto.dc(-Gyro_Power - Gyro_Turn_Error)
            Gyro_Error_Old = Gyro_Error_New
    if mode == 1:
        # Left wheel pivot turn
        Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
        while not (-1 <= Gyro_Error_New <= 1 or Gyro_Time_.time() > Gyro_Time):
            Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
            if Gyro_Error_Old != Gyro_Error_New:
                Gyro_Time_.reset()
            Gyro_Power = Gyro_Error_New * Gyro_KP__One_Motor_Movement_ + (Gyro_Error_New - Gyro_Error_Old) * Gyro_KD__One_Motor_Movement_
            if Gyro_Power >= power:
                Gyro_Power = power
            if Gyro_Power <= -power:
                Gyro_Power = -power
            L_moto.dc(Gyro_Power)
            Gyro_Error_Old = Gyro_Error_New
    if mode == 2:
        # Right wheel pivot turn
        Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
        while not (-1 <= Gyro_Error_New <= 1 or Gyro_Time_.time() > Gyro_Time):
            Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
            if Gyro_Error_Old != Gyro_Error_New:
                Gyro_Time_.reset()
            Gyro_Power = Gyro_Error_New * Gyro_KP__One_Motor_Movement_ + (Gyro_Error_New - Gyro_Error_Old) * Gyro_KD__One_Motor_Movement_
            if Gyro_Power >= power:
                Gyro_Power = power
            if Gyro_Power <= -power:
                Gyro_Power = -power
            R_moto.dc(-Gyro_Power)
            Gyro_Error_Old = Gyro_Error_New
    if stop == 1:
        L_moto.hold()
        R_moto.hold()
    if stop == 2:
        L_moto.stop()
        R_moto.stop()
    if Gyro_Time_.time() > Gyro_Time:
        prime_hub.speaker.beep(500, 100)

def Gyro_Reset(direction):
    global Gyro_Relative_Position_
    prime_hub.imu.reset_heading(direction)
    Gyro_Relative_Position_ = direction

def setup_values():
    global White_Line_Value, Black_Line_Value, PD_KP, PD_KD, Gyro_Move_KP, Gyro_Move_KD, Gyro_KP__One_Motor_Movement_, Gyro_KD__One_Motor_Movement_, Gyro_KP__Dual_Motor_Movement_, Gyro_KD__Dual_Motor_Movement_, Gyro_Time, Gray_Value
    # Recommended voltage: above 8050, for more stable power outputx`
    print('Battery : ', prime_hub.battery.voltage(), '  /  ', prime_hub.battery.voltage() * 2 / 167, '%')
    # View default values: White 80 / Black 20
    White_Line_Value = 70
    Black_Line_Value = 20
    # Line Following Default KP 0.3 / KD 3
    PD_KP = 0.3
    PD_KD = 6
    # Gyro Default Movement KP 6 / KD 60
    Gyro_Move_KP = 6
    Gyro_Move_KD = 60
    # Gyro Turn Default KP 7 / KD 70 (One Motor)
    Gyro_KP__One_Motor_Movement_ = 7
    Gyro_KD__One_Motor_Movement_ = 70
    # Gyro Turn KP 5 / KD 100 (Dual Motor)
    Gyro_KP__Dual_Motor_Movement_ = 5
    Gyro_KD__Dual_Motor_Movement_ = 100
    # Gyro Time Default 1s (Force Exit on Stall)


    # Sets the maximum duration for gyro movement.
    # If the time limit is reached and the action is not completed
    # (motor stall), the action will be forcibly exited automatically.
    # Prevents the program from halting.
    Gyro_Time = 1000

def Gyro_Value_(target):
    return prime_hub.imu.heading() - 360 * round((prime_hub.imu.heading() - target) / 370)

def move_time(speed, time):
    Time_01.reset()
    while not Time_01.time() >= time:
        L_moto.dc(speed)
        R_moto.dc(speed)
    L_moto.hold()
    R_moto.hold()

def GyroMoveRelative(target, min_speed, max_speed, accel_dist, decel_distance, total_distance, stop):
    global Move_Delta, Gyro_Initial_Position, Gyro_Relative_Position_, _Move_Power, Gyro_Error_New, Gyro_Power, Gyro_Error_Old
    L_moto.reset_angle(0)
    R_moto.reset_angle(0)
    Move_Delta = 0
    Gyro_Initial_Position = Gyro_Relative_Position_
    while not Move_Delta >= total_distance:
        Move_Delta = (abs(L_moto.angle()) + abs(R_moto.angle())) * 2721 / 10000
        Gyro_Relative_Position_ = Gyro_Initial_Position + Move_Delta * target / total_distance
        # setting Acceleration/ Deceleration
        if Move_Delta > total_distance - decel_distance and decel_distance != 0:
            _Move_Power = abs(min_speed) + (abs(max_speed) - abs(min_speed)) * (total_distance - Move_Delta) / abs(decel_distance)
            if min_speed < 0 or max_speed < 0:
                _Move_Power = -_Move_Power
        elif Move_Delta < accel_dist and accel_dist != 0:
            _Move_Power = abs(min_speed) + (abs(max_speed) - abs(min_speed)) * Move_Delta / abs(accel_dist)
            if min_speed < 0 or max_speed < 0:
                _Move_Power = -_Move_Power
        else:
            _Move_Power = max_speed
        Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
        Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
        Gyro_Error_Old = Gyro_Error_New
        L_moto.dc(_Move_Power + Gyro_Power)
        R_moto.dc(_Move_Power - Gyro_Power)
    if stop == 1:
        L_moto.hold()
        R_moto.hold()
    if stop == 2:
        L_moto.stop()
        R_moto.stop()
    Gyro_Relative_Position_ = Gyro_Initial_Position + target

def GyroMove(minSpeed, maxSpeed, accelDist, decelDist, distance, mode, stop):
    global Move_Delta, _Move_Power, Gyro_Error_New, Gyro_Power, Gyro_Error_Old
    L_moto.reset_angle(0)
    R_moto.reset_angle(0)
    Move_Delta = 0
    while not Move_Delta >= distance:
        Move_Delta = (abs(L_moto.angle()) + abs(R_moto.angle())) * 2721 / 10000
        # setting Acceleration/ Deceleration
        if Move_Delta > decelDist - decelDist and decelDist != 0:
            _Move_Power = abs(minSpeed) + (abs(maxSpeed) - abs(minSpeed)) * (decelDist - Move_Delta) / abs(decelDist)
            if minSpeed < 0 or maxSpeed < 0:
                _Move_Power = -_Move_Power
        elif Move_Delta < accelDist and accelDist != 0:
            _Move_Power = abs(minSpeed) + (abs(maxSpeed) - abs(minSpeed)) * Move_Delta / abs(accelDist)
            if minSpeed < 0 or maxSpeed < 0:
                _Move_Power = -_Move_Power
        else:
            _Move_Power = maxSpeed
        Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
        Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
        Gyro_Error_Old = Gyro_Error_New
        L_moto.dc(_Move_Power + Gyro_Power)
        R_moto.dc(_Move_Power - Gyro_Power)
    if mode == 1:
        # wait for white
        _Move_Power = minSpeed
        while not (leftCS.reflection() >= White_Line_Value or rightCS.reflection() >= White_Line_Value):
            Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
            Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
            Gyro_Error_Old = Gyro_Error_New
            L_moto.dc(_Move_Power + Gyro_Power)
            R_moto.dc(_Move_Power - Gyro_Power)
    if mode == 2:
        # wait for black
        _Move_Power = minSpeed
        while not (leftCS.reflection() <= Black_Line_Value or rightCS.reflection() <= Black_Line_Value):
            Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
            Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
            Gyro_Error_Old = Gyro_Error_New
            L_moto.dc(_Move_Power + Gyro_Power)
            R_moto.dc(_Move_Power - Gyro_Power)
    if mode == 3:
        # wait for black
        _Move_Power = minSpeed
        while not (leftCS.reflection() <= Black_Line_Value and rightCS.reflection() <= Black_Line_Value):
            Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
            Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
            Gyro_Error_Old = Gyro_Error_New
            L_moto.dc(_Move_Power + Gyro_Power)
            R_moto.dc(_Move_Power - Gyro_Power)
    if stop == 1:
        L_moto.hold()
        R_moto.hold()
    if stop == 2:
        L_moto.stop()
        R_moto.stop()

def PD_Line_mm(junction, min_speed, max_speed, accel_dist, decel_dist, total_dist, mode, stop):
    global Move_Delta, _Move_Power, Color_L, Color_R, PD_Error_New, PD_Power, PD_Error_Old
    L_moto.reset_angle(0)
    R_moto.reset_angle(0)
    Move_Delta = 0
    while not Move_Delta >= total_dist:
        Move_Delta = (abs(L_moto.angle()) + abs(R_moto.angle())) * 2721 / 10000
        # setting Acceleration/ Deceleration
        if Move_Delta < accel_dist and accel_dist != 0:
            _Move_Power = abs(min_speed) + (abs(max_speed) - abs(min_speed)) * Move_Delta / abs(accel_dist)
            if min_speed < 0 or max_speed < 0:
                _Move_Power = -_Move_Power
        elif Move_Delta > total_dist - decel_dist and decel_dist != 0:
            _Move_Power = abs(min_speed) + (abs(max_speed) - abs(min_speed)) * (total_dist - Move_Delta) / abs(decel_dist)
            if min_speed < 0 or max_speed < 0:
                _Move_Power = -_Move_Power
        else:
            _Move_Power = max_speed
        Color_L = leftCS.reflection()
        Color_R = rightCS.reflection()
        if junction == 1:
            if leftCS.reflection() <= 60:
                PD_Error_New = Color_L - Color_R * 0.6
            else:
                if rightCS.reflection() <= 60:
                    PD_Error_New = Color_L * 0.6 - Color_R
                else:
                    PD_Error_New = Color_L - Color_R
        else:
            PD_Error_New = Color_L - Color_R
        PD_Power = PD_Error_New * PD_KP + (PD_Error_New - PD_Error_Old) * PD_KD
        L_moto.dc(_Move_Power + PD_Power)
        R_moto.dc(_Move_Power - PD_Power)
        PD_Error_Old = PD_Error_New
    if mode == 1:
        # wait until left sensor sees black
        while not leftCS.reflection() <= Black_Line_Value:
            _PD_Track(junction, min_speed, 0)
    else:
        if mode == 2:
            # wait until right sensor sees black
            while not rightCS.reflection()<= Black_Line_Value:
                _PD_Track(junction, min_speed, 0)
        else:
            if mode == 3:
                # left or right sees line
                while not (leftCS.reflection() <= Black_Line_Value or rightCS.reflection()<= Black_Line_Value):
                    _PD_Track(junction, min_speed, 0)
            else:
                pass
    if stop == 1:
        L_moto.hold()
        R_moto.hold()
    if stop == 2:
        L_moto.stop()
        R_moto.stop()

def _PD_Track(side, power, threshold):
    global PD_Error_New, PD_Power, PD_Error_Old
    if side == 1:
        PD_Error_New = Tracking_Sensor.reflection() * 0.6 - threshold
    else:
        PD_Error_New = threshold - Object_Color_Detection.reflection() * 0.6
    PD_Power = PD_Error_New * PD_KP + (PD_Error_New - PD_Error_Old) * PD_KD
    L_moto.dc(power + PD_Power)
    R_moto.dc(power - PD_Power)
    PD_Error_Old = PD_Error_New


Gyro_Reset(0)
gyro_turn(90,50,0,0)
