function handles = two_link_workcell_init(vrep,id)

handles = struct('id',id);

armJoints = -1*ones(2,1);
links = -1*ones(2,1);
forcesensor = -1;
% for i=1:2
% [res,armJoints(i)] = vrep.simxGetObjectHandle(id,['joint',num2str(i)],vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
% [res,links(i)] = vrep.simxGetObjectHandle(id,['link',num2str(i)],vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
% end
[res,armJoints(1)] = vrep.simxGetObjectHandle(id,'joint1',vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
[res,armJoints(2)] = vrep.simxGetObjectHandle(id,'joint2',vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
[res,links(1)] = vrep.simxGetObjectHandle(id,'link1',vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
[res,links(2)] = vrep.simxGetObjectHandle(id,'link2',vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
[res,forcesensor] = vrep.simxGetObjectHandle(id,'Force_sensor',vrep.simx_opmode_oneshot_wait);vrchk(vrep, res);
handles.armJoints = armJoints;
handles.links = links;
handles.forcesensor = forcesensor;

%% streaming
for i=1:2
vrep.simxGetJointPosition(id,armJoints(i),vrep.simx_opmode_streaming);vrchk(vrep, res, true);
vrep.simxGetObjectFloatParameter(id,armJoints(i),2012,vrep.simx_opmode_streaming);vrchk(vrep, res, true); 
vrep.simxGetJointForce(id,armJoints(i),vrep.simx_opmode_streaming);vrchk(vrep, res, true);
end
vrep.simxReadForceSensor(id,forcesensor,vrep.simx_opmode_streaming);vrchk(vrep, res, true);
vrep.simxGetObjectOrientation(id,forcesensor,-1,vrep.simx_opmode_streaming);vrchk(vrep, res, true);
% Make sure that all streaming data has reached the client at least once
vrep.simxGetPingTime(id);
end
