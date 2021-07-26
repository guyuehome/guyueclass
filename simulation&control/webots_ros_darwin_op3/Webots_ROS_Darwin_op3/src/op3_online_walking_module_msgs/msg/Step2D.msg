#2D StepData

geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg


# which leg to be used (left/right/no, see below)
uint8 moving_foot   

uint8 LEFT_FOOT_SWING  = 1 # Left foot constant
uint8 RIGHT_FOOT_SWING = 2 # Right foot constant
uint8 STANDING         = 3 # Standing constant
