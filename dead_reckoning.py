import umath
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait,StopWatch

DEBUG = 0

def move_to_goal(waypoints):
    hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
    hub.imu.reset_heading(0)
    hub.speaker.play_notes(["C4/4","C4/4","C4/4","R/4","C4/2","C4/2","C4/1"],tempo=120)

    #Define Motors and their port connections
    l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE,reset_angle=True)
    r_Motor = Motor(Port.B, Direction.CLOCKWISE, reset_angle=True)

    drive_base = DriveBase(l_Motor,r_Motor,wheel_diameter=55.5, axle_track=127)
    drive_base.use_gyro(True)
    curr_pos = waypoints[0]

    for w in range(len(waypoints)):
        hub.speaker.beep(duration=10)
        pos_des = waypoints[w]

        x_dist = pos_des[0] - curr_pos[0]
        y_dist = pos_des[1] - curr_pos[1]

        if DEBUG:
            print(f"{x_dist}   {y_dist}")

        angle = umath.atan2(y_dist,x_dist)
        angle = umath.degrees(angle)
        angle = angle - drive_base.angle()
        
        drive_base.turn(angle)
    

        c_dist = umath.sqrt(umath.pow(x_dist,2) + umath.pow(y_dist,2))
        c_dist = c_dist * 304.8

        if curr_pos != pos_des:
            drive_base.straight(c_dist)

        curr_dist = drive_base.distance()
        
        hub.speaker.beep(duration=10)
        curr_pos = pos_des

        if DEBUG:
            print(curr_pos)