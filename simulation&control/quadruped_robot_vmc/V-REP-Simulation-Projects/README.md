# V-REP-Simulation-Projects
Learning Robotics by Playing with V-REP
# NOTE: ALL of the projects are successfully run on V-REP 4.1 and MATLAB 2020a 
## The right way to use force sensor in V-REP （4 steps totally, the step 3 is crucial to guarantee the validity of force/torque data）
### 1. get force sensor handle  
`[res,forcesensor] = vrep.simxGetObjectHandle(id,'Force_sensor',vrep.simx_opmode_oneshot_wait);`
### 2. data streaming  
`vrep.simxReadForceSensor(id,forcesensor,vrep.simx_opmode_streaming);`
### 3. read the first available data  
```
while (1)  
   [res,state,f,tau] = vrep.simxReadForceSensor(id,forcesensor,vrep.simx_opmode_buffer);
   if res == vrep.simx_return_ok && state == 1
      break;
   else
      disp('force sensor data is not available!');
      pause(0.1);
   end
end
```
### 4. read force data periodically in control loop  
```
while vrep.simxGetConnectionId(id)~=-1
   [res,state,f,tau] = vrep.simxReadForceSensor(id,forcesensor,vrep.simx_opmode_buffer);
end
```
