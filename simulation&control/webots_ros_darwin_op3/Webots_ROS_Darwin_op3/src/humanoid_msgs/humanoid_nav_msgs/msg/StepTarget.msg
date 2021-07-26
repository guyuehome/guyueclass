# Target for a single stepping motion of a humanoid's leg

geometry_msgs/Pose2D pose   # step pose as relative offset to last leg
uint8 leg                   # which leg to use (left/right, see below)

uint8 right=0               # right support leg constant
uint8 left=1                # left support leg constant
