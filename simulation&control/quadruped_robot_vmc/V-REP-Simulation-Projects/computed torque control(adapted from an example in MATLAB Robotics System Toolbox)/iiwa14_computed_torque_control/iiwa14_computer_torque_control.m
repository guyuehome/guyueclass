clear;
clc;

%% connect to vrep
disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5); % connect to vrep server
if (id>-1)
    disp('Connected to remote API server');
    %%  simulaiton parameters setting
    % simulation period
    dt = 1e-3;
    vrep.simxSetFloatingParameter(id,vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_oneshot_wait);
    vrep.simxSynchronous(id,true);
    % start the simulation:
    vrep.simxStartSimulation(id,vrep.simx_opmode_oneshot_wait);
    
    %%  initialization
    vrep.simxSynchronousTrigger(id);
    handles = iiwa14_computer_torque_control_workcell_init(vrep,id);
    joint_handles = handles.armJoints;
    pause(0.3);
    q = zeros(7,1);
    qdot = zeros(7,1);
    % get init state
    qstr = '[ ';
    qdotstr = '[ ';
    for j=1:7
        [res,q(j)] = vrep.simxGetJointPosition(id,joint_handles(j),vrep.simx_opmode_buffer);
        [res,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j),2012,vrep.simx_opmode_buffer);
        qstr = [qstr,num2str(q(j)),' '];
        qdotstr = [qdotstr,num2str(qdot(j)),' '];
    end
    qstr = [qstr,']'];
    qdotstr = [qdotstr,']'];
    
    inittime = vrep.simxGetLastCmdTime(id);
    t = 0;
    time = [];
    
    %% precomputed trajectory and torques
    lbr = importrobot('iiwa14.urdf');
    lbr.DataFormat = 'row';
    lbr.Gravity = [0 0 -9.80];
    load lbr_waypoints.mat
    cdt = 0.001;
    tt = 0:cdt:5;
    [qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints,qWaypoints,tt);
    n = size(qDesired,1);
    tauFeedForward = zeros(n,7);
    for i = 1:n
        tauFeedForward(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
    end
    %% pd controller parameters
    weights = [0.3,0.8,0.6,0.6,0.2,0.1,0.1];
    Kp = 100*weights;
    Kd = 2* weights;
    
    % variables used for computed torque control
    feedForwardTorque = zeros(n, 7);
    pdTorque = zeros(n, 7);
    timePoints = zeros(n,1);
    Q = zeros(n,7);
    QDesired = zeros(n,7);
    Qdot = zeros(n,7);
    QdotDesired = zeros(n,7);
    tau_cmd = zeros(n,7);
    i=1;
    while vrep.simxGetConnectionId(id)~=-1
        % time update
        currentCmdTime = vrep.simxGetLastCmdTime(id);
        t = (currentCmdTime-inittime)/1000;
        time = [time,t];
        % get states feedback
        for j=1:7
            [res,q(j)] = vrep.simxGetJointPosition(id,joint_handles(j),vrep.simx_opmode_buffer);
            [res,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j),2012,vrep.simx_opmode_buffer);
        end
        % Find the corresponding index h in tauFeedForward vector for joint
        % state time stamp t.
        h = ceil((t+1e-8)/cdt);
        %         disp(num2str(h));
        if h>n
            break
        end
        % Log joint positions data.
        Q(i,:) = q';
        QDesired(i,:) = qDesired(h,:);
        % Log joint velocity
        Qdot(i,:) = qdot';
        QdotDesired(i,:) = qdotDesired(h,:);
        % Inquire feed-forward torque at the time when the joint state is
        % updated (Gazebo sim time).
        tau1 = tauFeedForward(h,:);
        % Log feed-forward torque.
        feedForwardTorque(i,:) = tau1;
        
        % Compute PD compensation torque based on joint position and velocity
        % errors.
        tau2 = Kp.*(qDesired(h,:) - q') + Kd.*(qdotDesired(h,:) - qdot');
        % Log PD torque.
        pdTorque(i,:) = tau2';
        
        % Combine the two torques.
        tau = tau1 + tau2;
        %         tau = tau1;
        tau_cmd(i,:) = tau';
        % Send torque to vrep
        for j=1:7
            if tau(j)<0
                set_vel = -99999;
                set_tau= -tau(j);
            else
                set_vel = 99999;
                set_tau = tau(j);
            end
            vrep.simxSetJointTargetVelocity(id,joint_handles(j),set_vel,vrep.simx_opmode_oneshot);
            vrep.simxSetJointForce(id,joint_handles(j),set_tau,vrep.simx_opmode_oneshot);
        end
        vrep.simxSynchronousTrigger(id);
        i = i+1;
    end
    % Now close the connection to V-REP:
    vrep.simxStopSimulation(id,vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server1');
end
vrep.delete(); % call the destructor!


index=1:length(tau_cmd)';
figure(1)
for i=1:7
    subplot(2,4,i)
    plot(dt*index,feedForwardTorque(:,i),dt*index,tau_cmd(:,i));
    legend('taudesired','taucmd');
    xlabel('t/s');
    ylabel('joint torque/N.m');
end
suptitle('joint torque');

figure(2)
for i=1:7
    subplot(2,4,i)
    plot(dt*index,QDesired(:,i),dt*index,Q(:,i));
    legend('qdesired','qreal');
    xlabel('t/s');
    ylabel('position/rad');
end
suptitle('joint position');

figure(3)
for i=1:7
    subplot(2,4,i)
    plot(dt*index,QdotDesired(:,i),dt*index,Qdot(:,i));
    legend('qdotdesired','qdotreal');
    xlabel('t/s');
    ylabel('velocity/rad');
end
suptitle('joint velocity');