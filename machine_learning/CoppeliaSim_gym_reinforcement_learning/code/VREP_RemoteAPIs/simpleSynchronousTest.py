# This small example illustrates how to use the remote API
# synchronous mode. The synchronous mode needs to be
# pre-enabled on the server side. You would do this by
# starting the server (e.g. in a child script) with:
#
# simRemoteApi.start(19999,1300,false,true)
#
# But in this example we try to connect on port
# 19997 where there should be a continuous remote API
# server service already running and pre-enabled for
# synchronous mode.
#
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

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

import time
import sys

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # enable the synchronous mode on the client:
    sim.simxSynchronous(clientID,True)

    # start the simulation:
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Now step a few times:
    for i in range(1,10):
        if sys.version_info[0] == 3:
            input('Press <enter> key to step the simulation!')
        else:
            raw_input('Press <enter> key to step the simulation!')
        sim.simxSynchronousTrigger(clientID);

    # stop the simulation:
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
