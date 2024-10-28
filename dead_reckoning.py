import math
import time
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait

BASE_SPEED = 0.5
MAX_SPEED = 1.0

def dead_reckoning(hub=PrimeHub,l_motor=Motor,r_motor=Motor, pos_x=float, pos_y=float, theta=float):
    robot_heading = hub.imu.heading
    l_speed = l_motor.speed
    r_speed = r_motor.speed
    theta_dot = robot_heading-theta
    wheel_radius = 1.0

    x_dot = (wheel_radius*l_speed + wheel_radius*r_speed) * math.cos(theta)/2
    y_dot = (wheel_radius*l_speed + wheel_radius*r_speed) * math.sin(theta)/2

    pos_x += x_dot
    pos_y += y_dot
    theta += theta_dot

    return pos_x, pos_y, theta

class PID:
    def __init__(self, kp, ki, kd, desired_angle, sample_time=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = desired_angle  
        self.sample_time = sample_time

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def update(self, curr_angle, curr_time):
        # calculate angle error
        error = self.setpoint - curr_angle

        # normalize error to be in range of [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi

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
        self.setpoint = setpoint
        self.integral = 0.0

def move_to_goal(waypoints=list):
    hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)

    #Define Motors and their port connections
    l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE)
    r_Motor = Motor(Port.B, Direction.CLOCKWISE)

    pid = PID(kp=1.0, ki=0.1, kd=0.05, desired_angle=0)
    start_x, start_y = waypoints.pop()
    curr_pos = dead_reckoning(hub, l_Motor, r_Motor, start_x, start_y,0.0)
    curr_time = time.time()

    curr_waypoint_idx = 0
    while curr_waypoint_idx < len(waypoints):
        x_des, y_des = waypoints[curr_waypoint_idx]

        desired_angle = math.atan2(curr_pos[1]-y_des,curr_pos[0]-x_des)
        pid.set_target(desired_angle)
        curr_angle = hub.imu.heading* (math.pi*180)

        pid_output = pid.update(curr_angle, curr_time)
        angle_error = desired_angle - curr_angle
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        if angle_error > 0:
            lmotor_speed = BASE_SPEED - pid_output
            rmotor_speed = BASE_SPEED + pid_output
        else:
            lmotor_speed = BASE_SPEED + pid_output
            rmotor_speed = BASE_SPEED - pid_output

        lmotor_speed = max(min(lmotor_speed, MAX_SPEED), -MAX_SPEED)
        rmotor_speed = max(min(rmotor_speed, MAX_SPEED), -MAX_SPEED)

        # next waypoint if robot reached the current waypoint
        curr_pos = dead_reckoning(hub, l_Motor, r_Motor, curr_pos[0], curr_pos[1], curr_angle)
        if curr_pos[0] == x_des and curr_pos[1] == y_des:
            curr_waypoint_idx += 1 



