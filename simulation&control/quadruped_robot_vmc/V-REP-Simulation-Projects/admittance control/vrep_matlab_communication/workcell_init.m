function handles = workcell_init(vrep,id)
robot_name = 'LBR_iiwa_14_R820';

handles = struct('id',id);
%% arm joints
armJoints = -ones(1,7);
for i=1:7
    [res,armJoints(i)] = vrep.simxGetObjectHandle(id,[robot_name,'_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
end
handles.armJoints = armJoints;
%% sensors
% [res,external_force_sensor] = vrep.simxGetObjectHandle(id,[robot_name,'_connection'],vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
% handles.external_force_sensor = external_force_sensor;

%% streaming
for i=1:7
    vrep.simxGetJointPosition(id,armJoints(i),vrep.simx_opmode_streaming);vrchk(vrep, res, true); % joint position
    vrep.simxGetObjectFloatParameter(id,armJoints(i),2012,vrep.simx_opmode_streaming);vrchk(vrep, res, true); % joint velocity
    vrep.simxGetJointForce(id,armJoints(i),vrep.simx_opmode_streaming);vrchk(vrep, res, true); % joint torque
    vrep.simxGetStringSignal(id,'ContactForce',vrep.simx_opmode_streaming);vrchk(vrep, res); % contact force
end
% vrep.simxReadForceSensor(id,external_force_sensor,vrep.simx_opmode_streaming);vrchk(vrep, res, true);
% Make sure that all streaming data has reached the client at least once
vrep.simxGetPingTime(id);
end