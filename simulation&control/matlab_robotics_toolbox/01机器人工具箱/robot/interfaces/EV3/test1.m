b.outputPower(0,Device.MotorA, -10)
b.outputStart(0, Device.MotorA)
pause(1)
b.outputPower(0,Device.MotorA, -20)
pause(1)
b.outputPower(0,Device.MotorA, -10)
pause(1)

b.outputStop(0, Device.MotorA, 0)