function handles = Quadrotor_workcell_init(vrep,id)

robot_name = 'Quadrotor';
[res,Quadrotor] = vrep.simxGetObjectHandle(id,robot_name,vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
handles.Quadrotor = Quadrotor;

%% streaming
vrep.simxGetObjectPosition(id,Quadrotor,-1,vrep.simx_opmode_streaming);vrchk(vrep, res, true);
vrep.simxGetObjectOrientation(id,Quadrotor,-1,vrep.simx_opmode_streaming);vrchk(vrep, res, true);
vrep.simxGetObjectVelocity(id,Quadrotor,vrep.simx_opmode_streaming);vrchk(vrep, res, true);
% Make sure that all streaming data has reached the client at least once
vrep.simxGetPingTime(id);
end
