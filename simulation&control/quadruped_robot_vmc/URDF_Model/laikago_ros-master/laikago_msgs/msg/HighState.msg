uint8 levelFlag
uint8 mode
IMU imu
float32 forwardSpeed
float32 sideSpeed
float32 rotateSpeed
float32 bodyHeight
float32 updownSpeed
Cartesian forwardPosition       # (will be float type next version)
Cartesian sidePosition          # (will be float type next version)
Cartesian[4] footPosition2Body
Cartesian[4] footSpeed2Body
float32[4] footForce    
uint32 tick               
uint8[40] wirelessRemote
uint32 crc

# Under are not defined in SDK yet.
Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] jointP              # for visualization
