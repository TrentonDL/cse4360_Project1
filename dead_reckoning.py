import math
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait

def dead_reckoning(hub=PrimeHub,l_motor=Motor,r_motor=Motor, pos_x=float, pos_y=float, theta=float):
    robot_heading = hub.imu.heading
    l_speed = l_motor.speed
    r_speed = r_motor.speed
    theta_dot = robot_heading-theta
    wheel_radius = 1.0
    b = 5.0

    x_dot = (wheel_radius*l_speed + wheel_radius*r_speed) * math.cos(theta)/2
    y_dot = (wheel_radius*l_speed + wheel_radius*r_speed) * math.sin(theta)/2

    pos_x += x_dot
    pos_y += y_dot
    theta += theta_dot

    return pos_x, pos_y, theta


