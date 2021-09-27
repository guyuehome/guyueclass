# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/movementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script
#
# The client side (i.e. this script) depends on:
#
# sim.py, simConst.py, and the remote API library available
# in programming/remoteApiBindings/lib/lib
# Additionally you will need the python math and msgpack modules

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import math
import msgpack

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    executedMovId='notReady'

    targetArm='threadedBlueArm'
    #targetArm='nonThreadedRedArm'

    stringSignalName=targetArm+'_executedMovId'

    def waitForMovementExecuted(id):
        global executedMovId
        global stringSignalName
        while executedMovId!=id:
            retCode,s=sim.simxGetStringSignal(clientID,stringSignalName,sim.simx_opmode_buffer)
            if retCode==sim.simx_return_ok:
                executedMovId=s

    # Start streaming stringSignalName string signal:
    sim.simxGetStringSignal(clientID,stringSignalName,sim.simx_opmode_streaming)

    # Set-up some movement variables:
    mVel=100*math.pi/180
    mAccel=150*math.pi/180
    maxVel=[mVel,mVel,mVel,mVel,mVel,mVel]
    maxAccel=[mAccel,mAccel,mAccel,mAccel,mAccel,mAccel]
    targetVel=[0,0,0,0,0,0]

    # Start simulation:
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Wait until ready:
    waitForMovementExecuted('ready') 

    # Send first movement sequence:
    targetConfig=[90*math.pi/180,90*math.pi/180,-90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
    movementData={"id":"movSeq1","type":"mov","targetConfig":targetConfig,"targetVel":targetVel,"maxVel":maxVel,"maxAccel":maxAccel}
    packedMovementData=msgpack.packb(movementData)
    sim.simxCallScriptFunction(clientID,targetArm,sim.sim_scripttype_childscript,'legacyRapiMovementDataFunction',[],[],[],packedMovementData,sim.simx_opmode_oneshot)

    # Execute first movement sequence:
    sim.simxCallScriptFunction(clientID,targetArm,sim.sim_scripttype_childscript,'legacyRapiExecuteMovement',[],[],[],'movSeq1',sim.simx_opmode_oneshot)
    
    # Wait until above movement sequence finished executing:
    waitForMovementExecuted('movSeq1')

    # Send second and third movement sequence, where third one should execute immediately after the second one:
    targetConfig=[-90*math.pi/180,45*math.pi/180,90*math.pi/180,135*math.pi/180,90*math.pi/180,90*math.pi/180]
    targetVel=[-60*math.pi/180,-20*math.pi/180,0,0,0,0]
    movementData={"id":"movSeq2","type":"mov","targetConfig":targetConfig,"targetVel":targetVel,"maxVel":maxVel,"maxAccel":maxAccel}
    packedMovementData=msgpack.packb(movementData)
    sim.simxCallScriptFunction(clientID,targetArm,sim.sim_scripttype_childscript,'legacyRapiMovementDataFunction',[],[],[],packedMovementData,sim.simx_opmode_oneshot)
    targetConfig=[0,0,0,0,0,0]
    targetVel=[0,0,0,0,0,0]
    movementData={"id":"movSeq3","type":"mov","targetConfig":targetConfig,"targetVel":targetVel,"maxVel":maxVel,"maxAccel":maxAccel}
    packedMovementData=msgpack.packb(movementData)
    sim.simxCallScriptFunction(clientID,targetArm,sim.sim_scripttype_childscript,'legacyRapiMovementDataFunction',[],[],[],packedMovementData,sim.simx_opmode_oneshot)

    # Execute second and third movement sequence:
    sim.simxCallScriptFunction(clientID,targetArm,sim.sim_scripttype_childscript,'legacyRapiExecuteMovement',[],[],[],'movSeq2',sim.simx_opmode_oneshot)
    sim.simxCallScriptFunction(clientID,targetArm,sim.sim_scripttype_childscript,'legacyRapiExecuteMovement',[],[],[],'movSeq3',sim.simx_opmode_oneshot)
    
    # Wait until above 2 movement sequences finished executing:
    waitForMovementExecuted('movSeq3')
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    sim.simxGetStringSignal(clientID,stringSignalName,sim.simx_opmode_discontinue)
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

