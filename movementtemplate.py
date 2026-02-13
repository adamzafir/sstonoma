from definitions import *

def Move_mm(adjustment, minSpeed, maxSpeed, accelDist, decelDist, distance, mode, stop):
    global Move_Delta, _Move_Power
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    Move_Delta = 0
    while not Move_Delta >= distance:
        Move_Delta = (abs(left_motor.angle()) + abs(right_motor.angle())) * 2721 / 10000
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
        left_motor.dc(_Move_Power + adjustment)
        right_motor.dc(_Move_Power - adjustment)
    if mode == 1:
        _Move_Power = minSpeed
        while not left_sensor.reflection() >= White_Line_Value or right_sensor.reflection() >= White_Line_Value:
            left_motor.dc(_Move_Power + adjustment)
            right_motor.dc(_Move_Power - adjustment)
    if mode == 2:
        _Move_Power = minSpeed
        while not left_sensor.reflection() <= Black_Line_Value or right_sensor.reflection() <= Black_Line_Value:
            left_motor.dc(_Move_Power + adjustment)
            right_motor.dc(_Move_Power - adjustment)
    if stop == 1:
        left_motor.hold()
        right_motor.hold()
    if stop == 2:
        left_motor.stop()
        right_motor.stop()

def gyro_turn(target, power, mode, stop):
    global Gyro_Turn_Error, Gyro_Relative_Position_, Gyro_Error_New, Gyro_Power, Gyro_Error_Old
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    Gyro_Time_.reset()
    Gyro_Turn_Error = 0
    Gyro_Relative_Position_ = Gyro_Relative_Position_ + target
    if mode == 0:
        Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
        while not (-1 <= Gyro_Error_New <= 1 or Gyro_Time_.time() > Gyro_Time):
            Gyro_Turn_Error = (left_motor.angle() + right_motor.angle()) * 0.5
            Gyro_Error_New = round(Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_))
            if Gyro_Error_Old != Gyro_Error_New:
                Gyro_Time_.reset()
            Gyro_Power = Gyro_Error_New * Gyro_KP__Dual_Motor_Movement_ + (Gyro_Error_New - Gyro_Error_Old) * Gyro_KD__Dual_Motor_Movement_
            if Gyro_Power >= power:
                Gyro_Power = power
            if Gyro_Power <= -power:
                Gyro_Power = -power
            left_motor.dc(Gyro_Power - Gyro_Turn_Error)
            right_motor.dc(-Gyro_Power - Gyro_Turn_Error)
            Gyro_Error_Old = Gyro_Error_New
    if mode == 1:
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
            left_motor.dc(Gyro_Power)
            Gyro_Error_Old = Gyro_Error_New
    if mode == 2:
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
            right_motor.dc(-Gyro_Power)
            Gyro_Error_Old = Gyro_Error_New
    if stop == 1:
        left_motor.hold()
        right_motor.hold()
    if stop == 2:
        left_motor.stop()
        right_motor.stop()
    if Gyro_Time_.time() > Gyro_Time:
        hub.speaker.beep(500, 100)

def Gyro_Reset(direction):
    global Gyro_Relative_Position_
    hub.imu.reset_heading(direction)
    Gyro_Relative_Position_ = direction

def setup_values():
    global White_Line_Value, Black_Line_Value, PD_KP, PD_KD, Gyro_Move_KP, Gyro_Move_KD, Gyro_KP__One_Motor_Movement_, Gyro_KD__One_Motor_Movement_, Gyro_KP__Dual_Motor_Movement_, Gyro_KD__Dual_Motor_Movement_, Gyro_Time, Gray_Value
    print('Battery : ', hub.battery.voltage(), '  /  ', hub.battery.voltage() * 2 / 167, '%')
    White_Line_Value = 70
    Black_Line_Value = 20
    PD_KP = 0.3
    PD_KD = 6
    Gyro_Move_KP = 6
    Gyro_Move_KD = 60
    Gyro_KP__One_Motor_Movement_ = 7
    Gyro_KD__One_Motor_Movement_ = 70
    Gyro_KP__Dual_Motor_Movement_ = 5
    Gyro_KD__Dual_Motor_Movement_ = 100
    Gyro_Time = 1000

def Gyro_Value_(target):
    return hub.imu.heading() - 360 * round((hub.imu.heading() - target) / 370)

def move_time(speed, time):
    Time_01.reset()
    while not Time_01.time() >= time:
        left_motor.dc(speed)
        right_motor.dc(speed)
    left_motor.hold()
    right_motor.hold()

def GyroMoveRelative(target, min_speed, max_speed, accel_dist, decel_distance, total_distance, stop):
    global Move_Delta, Gyro_Initial_Position, Gyro_Relative_Position_, _Move_Power, Gyro_Error_New, Gyro_Power, Gyro_Error_Old
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    Move_Delta = 0
    Gyro_Initial_Position = Gyro_Relative_Position_
    while not Move_Delta >= total_distance:
        Move_Delta = (abs(left_motor.angle()) + abs(right_motor.angle())) * 2721 / 10000
        Gyro_Relative_Position_ = Gyro_Initial_Position + Move_Delta * target / total_distance
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
        left_motor.dc(_Move_Power + Gyro_Power)
        right_motor.dc(_Move_Power - Gyro_Power)
    if stop == 1:
        left_motor.hold()
        right_motor.hold()
    if stop == 2:
        left_motor.stop()
        right_motor.stop()
    Gyro_Relative_Position_ = Gyro_Initial_Position + target

def GyroMove(minSpeed, maxSpeed, accelDist, decelDist, distance, mode, stop):
    global Move_Delta, _Move_Power, Gyro_Error_New, Gyro_Power, Gyro_Error_Old
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    Move_Delta = 0
    while not Move_Delta >= distance:
        Move_Delta = (abs(left_motor.angle()) + abs(right_motor.angle())) * 2721 / 10000
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
        left_motor.dc(_Move_Power + Gyro_Power)
        right_motor.dc(_Move_Power - Gyro_Power)
    if mode == 1:
        _Move_Power = minSpeed
        while not (left_sensor.reflection() >= White_Line_Value or right_sensor.reflection() >= White_Line_Value):
            Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
            Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
            Gyro_Error_Old = Gyro_Error_New
            left_motor.dc(_Move_Power + Gyro_Power)
            right_motor.dc(_Move_Power - Gyro_Power)
    if mode == 2:
        _Move_Power = minSpeed
        while not (left_sensor.reflection() <= Black_Line_Value or right_sensor.reflection() <= Black_Line_Value):
            Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
            Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
            Gyro_Error_Old = Gyro_Error_New
            left_motor.dc(_Move_Power + Gyro_Power)
            right_motor.dc(_Move_Power - Gyro_Power)
    if mode == 3:
        _Move_Power = minSpeed
        while not (left_sensor.reflection() <= Black_Line_Value and right_sensor.reflection() <= Black_Line_Value):
            Gyro_Error_New = Gyro_Relative_Position_ - Gyro_Value_(Gyro_Relative_Position_)
            Gyro_Power = Gyro_Error_New * Gyro_Move_KP + (Gyro_Error_New - Gyro_Error_Old) * Gyro_Move_KD
            Gyro_Error_Old = Gyro_Error_New
            left_motor.dc(_Move_Power + Gyro_Power)
            right_motor.dc(_Move_Power - Gyro_Power)
    if stop == 1:
        left_motor.hold()
        right_motor.hold()
    if stop == 2:
        left_motor.stop()
        right_motor.stop()

def PD_Line_mm(junction, min_speed, max_speed, accel_dist, decel_dist, total_dist, mode, stop):
    global Move_Delta, _Move_Power, Color_L, Color_R, PD_Error_New, PD_Power, PD_Error_Old
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    Move_Delta = 0
    while not Move_Delta >= total_dist:
        Move_Delta = (abs(left_motor.angle()) + abs(right_motor.angle())) * 2721 / 10000
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
        Color_L = left_sensor.reflection()
        Color_R = right_sensor.reflection()
        if junction == 1:
            if left_sensor.reflection() <= 60:
                PD_Error_New = Color_L - Color_R * 0.6
            else:
                if right_sensor.reflection() <= 60:
                    PD_Error_New = Color_L * 0.6 - Color_R
                else:
                    PD_Error_New = Color_L - Color_R
        else:
            PD_Error_New = Color_L - Color_R
        PD_Power = PD_Error_New * PD_KP + (PD_Error_New - PD_Error_Old) * PD_KD
        left_motor.dc(_Move_Power + PD_Power)
        right_motor.dc(_Move_Power - PD_Power)
        PD_Error_Old = PD_Error_New
    if mode == 1:
        while not left_sensor.reflection() <= Black_Line_Value:
            _PD_Track(junction, min_speed, 0)
    else:
        if mode == 2:
            while not right_sensor.reflection() <= Black_Line_Value:
                _PD_Track(junction, min_speed, 0)
        else:
            if mode == 3:
                while not (left_sensor.reflection() <= Black_Line_Value or right_sensor.reflection() <= Black_Line_Value):
                    _PD_Track(junction, min_speed, 0)
            else:
                pass
    if stop == 1:
        left_motor.hold()
        right_motor.hold()
    if stop == 2:
        left_motor.stop()
        right_motor.stop()

def _PD_Track(side, power, threshold):
    global PD_Error_New, PD_Power, PD_Error_Old
    if side == 1:
        PD_Error_New = left_sensor.reflection() * 0.6 - threshold
    else:
        PD_Error_New = threshold - Object_Color_Detection.reflection() * 0.6
    PD_Power = PD_Error_New * PD_KP + (PD_Error_New - PD_Error_Old) * PD_KD
    left_motor.dc(power + PD_Power)
    right_motor.dc(power - PD_Power)
    PD_Error_Old = PD_Error_New