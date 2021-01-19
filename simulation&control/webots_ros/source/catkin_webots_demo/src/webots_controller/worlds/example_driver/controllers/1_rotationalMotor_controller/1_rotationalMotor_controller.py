# -*- coding: utf-8 -*-
"""motor_controller controller."""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor = robot.getMotor('my_Rmotor')

motor.setPosition(float('inf'))
motor.setVelocity(1)

# Main loop:
while robot.step(timestep) != -1:
    pass