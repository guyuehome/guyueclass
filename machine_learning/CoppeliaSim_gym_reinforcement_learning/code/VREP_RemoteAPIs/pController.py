# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/pControllerViaRemoteApi.ttt
#
# Do not launch simulation, but run this script
#
# The client side (i.e. this script) depends on:
#
# sim.py, simConst.py, and the remote API library available
# in programming/remoteApiBindings/lib/lib

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

class Client:
    def __enter__(self):
        self.intSignalName='legacyRemoteApiStepCounter'
        self.stepCounter=0
        self.maxForce=100
        sim.simxFinish(-1) # just in case, close all opened connections
        self.id=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
        return self
    
    def __exit__(self,*err):
        sim.simxFinish(-1)

with Client() as client:
    print("running")

    if client.id!=-1:
        print ('Connected to remote API server')

        def stepSimulation():
            currentStep=client.stepCounter
            sim.simxSynchronousTrigger(client.id);
            while client.stepCounter==currentStep:
                retCode,s=sim.simxGetIntegerSignal(client.id,client.intSignalName,sim.simx_opmode_buffer)
                if retCode==sim.simx_return_ok:
                    client.stepCounter=s

        def getCurrentJointAngle():
            retCode=-1
            jointA=0
            while retCode!=sim.simx_return_ok:
                retCode,jointA=sim.simxGetJointPosition(client.id,client.jointHandle,sim.simx_opmode_buffer)
            return jointA
        
        def moveToAngle(targetAngle):
            jointAngle=getCurrentJointAngle()
            while abs(jointAngle-targetAngle)>0.1*math.pi/180:
                stepSimulation()
                jointAngle=getCurrentJointAngle()
                vel=computeTargetVelocity(jointAngle,targetAngle)
                sim.simxSetJointTargetVelocity(client.id,client.jointHandle,vel,sim.simx_opmode_oneshot)
                sim.simxSetJointMaxForce(client.id,client.jointHandle,client.maxForce,sim.simx_opmode_oneshot)

        def computeTargetVelocity(jointAngle,targetAngle):
            dynStepSize=0.005
            velUpperLimit=360*math.pi/180
            PID_P=0.1
            errorValue=targetAngle-jointAngle
            sinAngle=math.sin(errorValue)
            cosAngle=math.cos(errorValue)
            errorValue=math.atan2(sinAngle,cosAngle)
            ctrl=errorValue*PID_P
            
            # Calculate the velocity needed to reach the position in one dynamic time step:
            velocity=ctrl/dynStepSize
            if (velocity>velUpperLimit):
                velocity=velUpperLimit
                
            if (velocity<-velUpperLimit):
                velocity=-velUpperLimit
            
            return velocity
        
         # Start streaming client.intSignalName integer signal, that signals when a step is finished:
        sim.simxGetIntegerSignal(client.id,client.intSignalName,sim.simx_opmode_streaming)
        
        res,client.jointHandle=sim.simxGetObjectHandle(client.id,'joint',sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client.id,client.jointHandle,360*math.pi/180,sim.simx_opmode_oneshot)
        sim.simxGetJointPosition(client.id,client.jointHandle,sim.simx_opmode_streaming)
        
        # enable the synchronous mode on the client:
        sim.simxSynchronous(client.id,True)
        
        sim.simxStartSimulation(client.id,sim.simx_opmode_oneshot)
        
        moveToAngle(45*math.pi/180)
        moveToAngle(90*math.pi/180)
        moveToAngle(-89*math.pi/180) #no -90, to avoid passing below
        moveToAngle(0*math.pi/180)
        
        sim.simxStopSimulation(client.id,sim.simx_opmode_blocking)
