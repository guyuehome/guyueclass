import sys
sys.path.append('../VREP_RemoteAPIs')

import sim as vrep_sim

# CartPole simulation model in CoppeliaSim
class CartPoleModel():
    def __init__(self):
        super(self.__class__, self).__init__()
        self.client_ID = None
        self.joint_1_handle = None
        self.joint_2_handle = None

    def initializeSimModel(self, client_ID):
        try:
            print ('Connected to remote API server')
            client_ID != -1
        except:
            print ('Failed connecting to remote API server')

        self.client_ID = client_ID

        return_code, self.joint_1_handle = vrep_sim.simxGetObjectHandle(client_ID, 'joint_1', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object joint 1 ok.')

        return_code, self.joint_2_handle = vrep_sim.simxGetObjectHandle(client_ID, 'joint_2', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object joint 2 ok.')

        # Get the joint position
        return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_1_handle, vrep_sim.simx_opmode_streaming)
        return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_2_handle, vrep_sim.simx_opmode_streaming)
        self.setJointTorque(0)
    
    def getJointPosition(self, joint_name):
        """
        :param: joint_name: string
        """
        q = 0
        if joint_name == 'joint_1':
            _, q = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_1_handle, vrep_sim.simx_opmode_buffer)
        elif joint_name == 'joint_2':
            _, q = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_2_handle, vrep_sim.simx_opmode_buffer)
        else:
            print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')

        return q

    def setJointTorque(self, torque):
        if torque >= 0:
            vrep_sim.simxSetJointTargetVelocity(self.client_ID, self.joint_1_handle, 1000, vrep_sim.simx_opmode_oneshot)
        else:
            vrep_sim.simxSetJointTargetVelocity(self.client_ID, self.joint_1_handle, -1000, vrep_sim.simx_opmode_oneshot)

        vrep_sim.simxSetJointMaxForce(self.client_ID, self.joint_1_handle, abs(torque), vrep_sim.simx_opmode_oneshot)
