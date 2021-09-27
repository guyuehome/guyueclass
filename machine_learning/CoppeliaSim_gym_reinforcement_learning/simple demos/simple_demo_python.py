import time
import numpy as np
import sys
sys.path.append('./VREP_RemoteAPIs/')
import sim as vrep_sim

print('\n--------- Program start from here ...')

# ------------------------------- Connect to CoppeliaSim ------------------------------- 
vrep_sim.simxFinish(-1) # just in case, close all opened connections
while True:
    client_ID = vrep_sim.simxStart('127.0.0.1', 19998, True, False, 5000, 5) # Connect to CoppeliaSim
    if client_ID > -1: # connected
        print('Connect to remote API server.')
        break
    else:
        print('Failed connecting to remote API server! Try it again ...')

# Open synchronous mode
synchronous_enable = True # Enable/Disable synchronous control
vrep_sim.simxSynchronous(client_ID, synchronous_enable)
vrep_sim.simxStartSimulation(client_ID, vrep_sim.simx_opmode_oneshot)

# ------------------------------- Initialization ------------------------------- 
return_code, joint_handle = vrep_sim.simxGetObjectHandle(client_ID, 'joint', vrep_sim.simx_opmode_blocking)
if (return_code == vrep_sim.simx_return_ok):
    print('get object joint ok.')

# Initialization
_, q = vrep_sim.simxGetJointPosition(client_ID, joint_handle, vrep_sim.simx_opmode_streaming)
vrep_sim.simxSetJointTargetPosition(client_ID, joint_handle, 0, vrep_sim.simx_opmode_streaming)

# ------------------------------- Simulation ------------------------------- 
t = 0
delta_t = 0.005 # simulation time step
for _ in range(5000):
    t = t + delta_t

    _, q = vrep_sim.simxGetJointPosition(client_ID, joint_handle, vrep_sim.simx_opmode_buffer)
    print('q=', q)

    q_new = np.sin(t)
    vrep_sim.simxSetJointTargetPosition(client_ID, joint_handle, q_new, vrep_sim.simx_opmode_streaming)

    #----- For synchronous control
    if synchronous_enable == True:
        vrep_sim.simxSynchronousTrigger(client_ID)
        _, ping_time = vrep_sim.simxGetPingTime(client_ID) # Ensure that all commands is finished

    time.sleep(delta_t)

vrep_sim.simxStopSimulation(client_ID, vrep_sim.simx_opmode_blocking)
vrep_sim.simxFinish(client_ID)

print('Program terminated')
