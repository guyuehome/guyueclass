uint8 levelFlag
IMU imu
MotorState[20] motorState
float32[4] footForce
uint32 tick                   # time reference
uint8[40] wirelessRemote      # wireless command
uint32 crc                    # check bit
Cartesian[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 