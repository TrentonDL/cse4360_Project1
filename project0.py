from pybricks.pupdevices import Motor
from pybricks.pupdevices import UltrasonicSensor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait

l_Motor = Motor(Port.C,Direction.COUNTERCLOCKWISE)
r_Motor = Motor(Port.B, Direction.CLOCKWISE)
ultra_sensor = UltrasonicSensor(Port.A)

while ultra_sensor.distance() > 50:
    if ultra_sensor.distance() >= 500:
        l_Motor.run(500)
        r_Motor.run(500)
    elif ultra_sensor.distance() < 500 and ultra_sensor.distance() > 200:
        ultra_sensor.lights.on(100)
        r_Motor.run(500)
        l_Motor.run(200)
    elif ultra_sensor.distance() <= 200:
        r_Motor.run(200)
        l_Motor.run(0)

l_Motor.run(0)
r_Motor.run(0)
ultra_sensor.lights.off()
