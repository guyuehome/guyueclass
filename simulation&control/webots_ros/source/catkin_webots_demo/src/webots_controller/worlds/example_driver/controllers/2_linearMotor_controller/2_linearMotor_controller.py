# -*- coding: utf-8 -*-
"""linear motor controller."""

from controller import Robot
import math


robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor = robot.getMotor('my_lMotor')
#motor.setPosition(float('inf'))
#motor.setVelocity(0)

# Main loop:
count = 0
while robot.step(timestep) != -1:
    #motor.setVelocity(1)
    motor.setPosition(math.sin(count)*0.5)
    count += 0.1