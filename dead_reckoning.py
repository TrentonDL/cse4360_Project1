import umath
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait,StopWatch

DEBUG = 0

# accept a list of coordinates as a Path and move through the path one coordinate at a time
def move_to_goal(waypoints):
    #initialize the hub to wear the Z axis is Up and the X Axis is going through the front
    hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
    hub.imu.reset_heading(0)

    # Warning Siren: Robot is about to move!
    hub.speaker.play_notes(["C4/4","C4/4","C4/4","R/4","C4/2","C4/2","C4/1"],tempo=120)

    #Define Motors and their port connections
    l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE,reset_angle=True)
    r_Motor = Motor(Port.B, Direction.CLOCKWISE, reset_angle=True)

    # Initialize Drive Base PID Controler with a wheel diameter and axle_track in mm 
    drive_base = DriveBase(l_Motor,r_Motor,wheel_diameter=55.5, axle_track=127)
    drive_base.use_gyro(True)

    # get the starting position
    curr_pos = waypoints[0]

    #loop through each coordinate/waypoint given in the path
    for w in range(len(waypoints)):
        hub.speaker.beep(duration=10)
        # new Waypoint
        pos_des = waypoints[w]

        #calculate difference in the waypoint and current position
        x_dist = pos_des[0] - curr_pos[0]
        y_dist = pos_des[1] - curr_pos[1]

        if DEBUG:
            print(f"{x_dist}   {y_dist}")

        # calculate the current angle and subtract it from the IMU angle to determine if the robot needs to rotate
        angle = umath.atan2(y_dist,x_dist)
        angle = umath.degrees(angle)
        angle = angle - drive_base.angle()
        
        #rotate robot at the set angle, (will be 0 if the robot does not need to turn)
        drive_base.turn(angle)
    
        # calculate the distance needed to move forward in case the path has a werid angle and convert to mm
        c_dist = umath.sqrt(umath.pow(x_dist,2) + umath.pow(y_dist,2))
        c_dist = c_dist * 304.8

        # drive forward if not at current position
        if curr_pos != pos_des:
            drive_base.straight(c_dist)

        curr_dist = drive_base.distance()
        
        # reached waypoint current position now is equal to position desired
        hub.speaker.beep(duration=10)
        curr_pos = pos_des

        if DEBUG:
            print(curr_pos)