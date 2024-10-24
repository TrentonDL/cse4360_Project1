import math
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait

def dead_reckoning(l_motor=Motor, r_motor=Motor, pos_x=float, pos_y=float, theta=float):
    l_speed = l_motor.speed
    r_speed = r_motor.speed
    wheel_radius = 1.0
    b = 5.0

    pos_x = (wheel_radius*l_speed + wheel_radius*r_speed) * math.cos(theta)/2
    pos_y = (wheel_radius*l_speed + wheel_radius*r_speed) * math.sin(theta)/2
    theta = (wheel_radius*l_speed - wheel_radius*r_speed) / b

    velocity = math.sqrt(math.pow(pos_x,2)+math.pow(pos_y,2))

    return [pos_x,pos_y,theta,velocity]


