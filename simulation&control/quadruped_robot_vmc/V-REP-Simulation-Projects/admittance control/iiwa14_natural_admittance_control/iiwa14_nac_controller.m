clear;
clc;
addpath(genpath('..'));
disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5); % connect to vrep server
% vrep.simxLoadScene(id,'iiwa14_nac_controll.ttt',1,vrep.simx_opmode_oneshot_wait); % add scene

if (id>-1)
    disp('Connected to remote API server');
    %%  simulaiton parameters setting
    % simulation period
    dt = 5e-3;
    % simulation duration you can make it larger if you'd like to delete
    % the little stone in V-REP during the simulation
    duration = 8;
    vrep.simxSetFloatingParameter(id,vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_oneshot_wait);
    % Make sure we close the connection whenever the script is interrupted.
%     cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
    % enable the synchronous mode on the client:
    vrep.simxSynchronous(id,true);
    % start the simulation:
    vrep.simxStartSimulation(id,vrep.simx_opmode_oneshot_wait);

    %%  initialization 
    vrep.simxSynchronousTrigger(id);
    handles = workcell_init(vrep,id);
    joint_handles = handles.armJoints;
%     external_force_sensor = handles.external_force_sensor;
    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or 
%     % the robot pose with the simx_opmode_buffer option.(this only works in asynchronous mode)
    pause(.3);
    % get simulation time
    currentCmdTime = vrep.simxGetLastCmdTime(id);
    lastCmdTime = currentCmdTime;
    % limits for joint position and velocity
    qmax = [170,120,170,120,170,120,175]*pi/180;
    qmin = -qmax;
    dqlimit = [110,110,128,128,204,184,184]*pi/180;
    % some variables for computation
    feedback_joint_position = zeros(7,1);
    feedback_joint_velocity = zeros(7,1);
    feedback_joint_torque = zeros(7,1);
    target_joint_position = zeros(7,1);
    target_joint_velocity = zeros(7,1);
%     external_force_vec = zeros(3,1);
%     external_torque_vec = zeros(3,1);
    contact_force = zeros(3,1);
    % get init state
    for i=1:7
        [res,feedback_joint_position(i)] = vrep.simxGetJointPosition(id,joint_handles(i),vrep.simx_opmode_buffer);
        [res,feedback_joint_velocity(i)] = vrep.simxGetObjectFloatParameter(id,joint_handles(i),2012,vrep.simx_opmode_buffer);
        [res,feedback_joint_torque(i)] = vrep.simxGetJointForce(id,joint_handles(i),vrep.simx_opmode_buffer);
    end
    % velocity calculated by difference
%     feedback_joint_velocity = (feedback_joint_position - lastConfiguration)/dt;
    target_joint_position = feedback_joint_position;
    target_joint_velocity = feedback_joint_velocity;
    % store last and current configuration
    lastConfiguration = feedback_joint_position;
    
    % get init contact force
    [res,contact_force_string] = vrep.simxGetStringSignal(id,'ContactForce',vrep.simx_opmode_buffer);
    contact_force = vrep.simxUnpackFloats(contact_force_string);
    contact_force = contact_force';    %
    robot_type = 1;
    [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
    target_end_effector_p = pose(1:3,4);
    target_end_effector_r = pose(1:3,1:3);
    % some variables for data record and plot
    v_cartesians = [];
    p_cartesians = [];
%     f_external_forces = [];
    f_contact_forces = [];
    f_attractors = [];
    f_nets = [];
    q_ddots = [];
    target_joint_positions = [];
    target_joint_velocities = [];
    feedback_joint_ps = [];
    feedback_joint_vs = [];
    feedback_joint_ts = [];
    %% natural admittance control
    % cartesian impedance controller parameters
    k_cartesian = diag([100,100,80]);
    b_cartesian = diag([150,150,120]);
    % equivalent inertial in joint space
    H_inv = diag([1 1 1 1 1 1 0]);
    % gain on the orientation difference of the end effector
    k_vel_p = 50;
    
    % trigger simulation
    vrep.simxSynchronousTrigger(id);
    % time record
    time = [];
    t = 0;
    % cyclic control
    while (vrep.simxGetConnectionId(id)~=-1)
        if t>duration
            break;
        end
        % time update
        currentCmdTime = vrep.simxGetLastCmdTime(id);
        dt = (currentCmdTime-lastCmdTime)/1000;
        % get states feedback
        for i=1:7
            [res,feedback_joint_position(i)] = vrep.simxGetJointPosition(id,joint_handles(i),vrep.simx_opmode_buffer);
            [res,feedback_joint_velocity(i)] = vrep.simxGetObjectFloatParameter(id,joint_handles(i),2012,vrep.simx_opmode_buffer);
            [res,feedback_joint_torque(i)] = vrep.simxGetJointForce(id,joint_handles(i),vrep.simx_opmode_buffer);
        end
%         feedback_joint_velocity = (feedback_joint_position-lastConfiguration)/dt;
        lastConfiguration = feedback_joint_position;
%         velocity_errors = [velocity_errors,target_joint_velocity - feedback_joint_velocity];
        % get external force in force sensor frame
%         [res,force_sensor_state,external_force_vec,external_torque_vec] = vrep.simxReadForceSensor(id,external_force_sensor,vrep.simx_opmode_buffer);
        [res,contact_force_string] = vrep.simxGetStringSignal(id,'ContactForce',vrep.simx_opmode_buffer);
        contact_force = vrep.simxUnpackFloats(contact_force_string);
        contact_force = contact_force';
        % controller
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,robot_type);
        J_dx_dq = Jac(1:3,:);
        
        [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose(1:3,4);
        v_cartesian = J_dx_dq*feedback_joint_velocity;
        f_attractor = k_cartesian*(target_end_effector_p-end_effector_p)-b_cartesian*v_cartesian;
        % transform external force to cartesian frame
%         external_force_vec = A_mat_products{7}(1:3,1:3)*external_force_vec';
%         f_net = f_attractor + external_force_vec;
        f_net = f_attractor + contact_force;
        J_transpose_f = J_dx_dq'*f_net;
        q_ddot = H_inv*(J_transpose_f-feedback_joint_velocity);
        target_joint_velocity = target_joint_velocity + q_ddot*dt;
%         target_joint_velocity = feedback_joint_velocity;
        [issingular,sols]=SolveSphericalWrist(feedback_joint_position,target_end_effector_r,robot_type);
        if size(sols,2)>0
            q_vec_err = sols(:,1) - feedback_joint_position;
            if size(sols,2)>1
                q_err1 = norm(q_vec_err);
                q_err2 = norm(sols(:,2) - feedback_joint_position);
                if q_err2<q_err1
                    q_vec_err = sols(:,2) - feedback_joint_position;
                end
            end
            for i=5:7
                target_joint_velocity(i) =  k_vel_p*q_vec_err(i);
                
            end
        end
        
        target_joint_velocity = satdq(dqlimit,target_joint_velocity);
        
        target_joint_position = feedback_joint_position + target_joint_velocity*dt;
        target_joint_position = satq(qmin,qmax,target_joint_position);
        % record data
        v_cartesians = [v_cartesians,v_cartesian];
        p_cartesians = [p_cartesians,end_effector_p];
%         f_external_forces = [f_external_forces,external_force_vec];
        f_contact_forces = [f_contact_forces,contact_force];
        f_attractors = [f_attractors,f_attractor];
        f_nets = [f_nets,f_net];
        q_ddots = [q_ddots,q_ddot];
        time = [time,t];
        target_joint_velocities = [target_joint_velocities,target_joint_velocity];
        target_joint_positions = [target_joint_positions,target_joint_position];
        feedback_joint_vs = [feedback_joint_vs,feedback_joint_velocity];
        feedback_joint_ps = [feedback_joint_ps,feedback_joint_position];
        feedback_joint_ts = [feedback_joint_ts,feedback_joint_torque];
        % send command
        for i=1:7
            res = vrep.simxSetJointTargetPosition(id, joint_handles(i),target_joint_position(i), vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        end
        % update vrep 
        lastCmdTime = currentCmdTime;
        vrep.simxSynchronousTrigger(id);
        t = t+dt;
    end
    % Now close the connection to V-REP:
    vrep.simxStopSimulation(id,vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server1');
end
vrep.delete(); % call the destructor!
% plot
figure(1); 
%p_cartesian
subplot(2,1,1)
hold on
for i=1:3
    plot(time,p_cartesians(i,:));
end
legend('x','y','z');
xlabel('t/s');
ylabel('p/m');
title('end effector position');
hold off

%v_cartesian
subplot(2,1,2);
hold on
for i=1:3
    plot(time,v_cartesians(i,:));
end
legend('vx','vy','vz');
xlabel('t/s');
ylabel('v/m.s-1');
title('end effector velocity');
hold off

figure(2); %f
subplot(3,1,1)
hold on
for i=1:3
%     plot(time,f_external_forces(i,:));
plot(time,f_contact_forces(i,:));
end
legend('fx','fy','fz');
xlabel('t/s');
ylabel('f/N');
title('external force');
hold off

subplot(3,1,2)
hold on
for i=1:3
    plot(time,f_attractors(i,:));
end
legend('fx','fy','fz');
xlabel('t/s');
ylabel('f/N');
title('virtual force');
hold off

subplot(3,1,3)
hold on
for i=1:3
    plot(time,f_nets(i,:));
end
legend('fx','fy','fz');
xlabel('t/s');
ylabel('f/N');
title('net force');
hold off

figure(3); %a
hold on
for i=1:7
    plot(time,q_ddots(i,:));
end
legend('j1a','j2a','j3a','j4a','j5a','j6a','j7a');
xlabel('t/s');
ylabel('a/m.s-2');
title('desired joint acceleration');
hold off

figure(4)
subplot(2,1,1)
hold on
for i=1:7
    plot(time,target_joint_velocities(i,:));
end
legend('j1a','j2a','j3a','j4a','j5a','j6a','j7a');
xlabel('t/s');
ylabel('v/rad.s-1');
title('target joint v');
hold off

subplot(2,1,2)
hold on
for i=1:7
    plot(time,feedback_joint_vs(i,:));
end
legend('j1a','j2a','j3a','j4a','j5a','j6a','j7a');
xlabel('t/s');
ylabel('v/rad.s-1');
title('feedback joint v');
hold off

figure(5)
hold on
for i=1:7
    plot(time,feedback_joint_ts(i,:));
end
legend('j1t','j2t','j3t','j4t','j5t','j6t','j7t');
xlabel('t/s');
ylabel('tau/N.m');
title('feedback joint torque');
hold off

% figure(6)
% subplot(2,1,1)
% hold on
% for i=1:7
%     plot(time,target_joint_positions(i,:));
% end
% legend('j1a','j2a','j3a','j4a','j5a','j6a','j7a');
% xlabel('t/s');
% ylabel('cmd joint p/rad');
% hold off
% 
% subplot(2,1,2)
% hold on
% for i=1:7
%     plot(time,feedback_joint_ps(i,:));
% end
% legend('j1a','j2a','j3a','j4a','j5a','j6a','j7a');
% xlabel('t/s');
% ylabel('feedback joint p/rad');
% hold off
disp('Program ended');