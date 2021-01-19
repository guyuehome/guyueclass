"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor1 = robot.getMotor('wheel1')
motor2 = robot.getMotor('wheel2')
motor3 = robot.getMotor('wheel3')
motor4 = robot.getMotor('wheel4')

motor1.setPosition(float('inf'))
motor1.setVelocity(2)
motor2.setPosition(float('inf'))
motor2.setVelocity(2)
motor3.setPosition(float('inf'))
motor3.setVelocity(2)
motor4.setPosition(float('inf'))
motor4.setVelocity(2)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
