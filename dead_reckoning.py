import umath
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait,StopWatch

BASE_SPEED = 100.0
MAX_SPEED = 500.0

def dead_reckoning(hub, l_motor, r_motor, pos_x, pos_y, theta, last_time, wheel_radius=1):
    # Current robot heading
    robot_heading = hub.imu.heading()
    
    # Get motor speeds and convert to radians per second
    l_speed = float(l_motor.speed())  # Assume this is in degrees per second
    r_speed = float(r_motor.speed())
    l_speed_rad = umath.radians(l_speed)  # Convert to radians/sec
    r_speed_rad = umath.radians(r_speed)
    
    # Calculate current time and delta time (time elapsed since last update)
    dt = last_time  # Time in seconds

    # Update theta to the current robot heading (convert heading to radians)
    robot_heading_rad = umath.radians(robot_heading)
    theta_rad = umath.radians(theta)
    theta_dot = robot_heading_rad - theta_rad
    theta += theta_dot  # Update theta based on robot heading

    # Calculate forward velocity
    velocity = wheel_radius * (l_speed_rad + r_speed_rad) / 2.0

    # Calculate changes in x and y using forward velocity and time elapsed
    x_dot = velocity * umath.cos(theta)
    y_dot = velocity * umath.sin(theta)
    
    # Update positions based on velocities and delta time
    pos_x += x_dot * float(dt)
    pos_y += y_dot * float(dt)

    # Convert theta back to degrees if required
    theta_deg = umath.degrees(theta)

    return pos_x, pos_y, theta_deg, dt  # Include curr_time for next update's reference

class PID:
    def __init__(self, kp, ki, kd, desired_angle, sample_time=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = desired_angle  
        self.sample_time = sample_time

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = StopWatch()

    def update(self, curr_angle, curr_time):
        # calculate angle error
        error = self.setpoint - curr_angle

        # normalize error to be in range of [-pi, pi]
        error = (error + umath.pi) % (2.0 * umath.pi) - umath.pi

        delta_time = curr_time - (self.prev_time if self.prev_time else curr_time)
        
        if delta_time >= self.sample_time:
            # proportional term
            p_term = self.kp * error
            
            # integral term
            self.integral += error * delta_time
            i_term = self.ki *self.integral

            # derivative term
            d_term = self.kd * (error - self.prev_error) / delta_time if delta_time > 0 else 0.0

            output = p_term + i_term + d_term

            # save error and time for next call
            self.prev_error = error
            self.prev_time = curr_time

            return output
        return 0.0 # if no sample time has elapsed return 0
    
    def set_target(self, setpoint):
        self.setpoint = umath.degrees(setpoint)
        self.integral = 0.0

def move_to_goal(waypoints):
    hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
    hub.imu.reset_heading(0)

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
        print(curr_pos)