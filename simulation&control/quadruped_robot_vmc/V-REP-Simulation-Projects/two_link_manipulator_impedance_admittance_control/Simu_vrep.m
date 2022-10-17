clear;
disp('Program started');
% control mode (0: impedance control
%               1: admittance control(velocity command)
%               2: admittance control(position command))
control_mode = 0;

% reference trajectory mode (0: horizontal line
%                            1: circle)
trajectory_mode = 0;

% dyanmic params(the same as those used in vrep)
I1 = 1;
I2 = 1;
l1 = 0.3;
l2 = 0.3;
m1 = 0.5;
m2 = 0.5;
a1 = 0.15;
a2 = 0.15;
g = 9.81;
% impedance model params
kpx = 150;
kpy = 150;
kdx = 70;
kdy = 70;
mx = 1;
my = 1;

% for moving avarage force filter
windowSize = 5;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
raw_externalforces = zeros(2,windowSize);
filtered_externalforce = zeros(2,1);

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5); % connect to vrep server
if (id>-1)
    disp('Connected to remote API server');
    % timestep
    dt = 0.01;
    vrep.simxSetFloatingParameter(id,vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_oneshot_wait);
    % set sychronous mode
    vrep.simxSynchronous(id,true);
    % start the simulation:
    vrep.simxStartSimulation(id,vrep.simx_opmode_oneshot_wait);
    % trigger one step simulation and pause a short time to clear dirty
    % data in vrep buffer
    vrep.simxSynchronousTrigger(id);
%     pause(0.3);
    
    handles = two_link_workcell_init(vrep,id);
    armJoints = handles.armJoints;
    forcesensor = handles.forcesensor;
    x0 = [0;0;0;0];
    tau = [0;0];
    
    % set joint control mode in vrep
    if control_mode == 2
        for i=1:2
          vrep.simxSetObjectIntParameter(id,armJoints(i),2001,1,vrep.simx_opmode_oneshot_wait);
        end
    else
        for i=1:2
          vrep.simxSetObjectIntParameter(id,armJoints(i),2001,0,vrep.simx_opmode_oneshot_wait);
        end
    end
    % get init joint states
    for i=1:2
        [res,x0(i)] = vrep.simxGetJointPosition(id,armJoints(i),vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        [res,x0(i+2)] = vrep.simxGetObjectFloatParameter(id,armJoints(i),2012,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        [res,tau(i)] = vrep.simxGetJointForce(id,armJoints(i),vrep.simx_opmode_buffer);vrchk(vrep, res, true);
    end
    % get external force in force sensor frame
    [res,state,fexternal,tauexternal] = vrep.simxReadForceSensor(id,forcesensor,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
    % transform external force into the robot base frame(here the robot base frame coincides with the world frame in vrep)
    [res,eulerAngles]=vrep.simxGetObjectOrientation(id,forcesensor,-1,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
    forcesensorrotation = rotx(double(eulerAngles(1)*180/pi))*roty(double(eulerAngles(2)*180/pi))*rotz(double(eulerAngles(3)*180/pi));
    fexternal_inertial = forcesensorrotation*[fexternal(1);fexternal(2);fexternal(3)];
    raw_externalforces(1,:) = fexternal_inertial(1)*ones(1,windowSize);
    raw_externalforces(2,:) = fexternal_inertial(3)*ones(1,windowSize);
    % record data
    recordData.x = [];
    recordData.tau = [];
    recordData.fmea = [];
    recordData.u = [];
    recordData.t = [];
    % max simulation duration
    maxsimtime = 20;
    % current time
    current_time = 0;
    % reference joint trajectory
    qref = x0(1:2);
    qdotref = [0;0];
    qdotdotref = [0;0];
    % reference ee trajectory
    eeposref = Direct_Kinematics(x0(1:2),[l1;l2]);
    eevref = [0;0];
    eearef = [0;0];
    % for impedance model
    xe_admittance = [0;0];
    xedot_admittance = [0;0];
    xedotdot_admittance = [0;0];
    q_admittance = [0;0];
    qdot_admittance = [0;0];
    qdotdot_admittance = [0;0];
    
    while vrep.simxGetConnectionId(id)~=-1
        % get robot joint states
        for i=1:2
            [res,x0(i)] = vrep.simxGetJointPosition(id,armJoints(i),vrep.simx_opmode_buffer);vrchk(vrep, res, true);
            [res,x0(i+2)] = vrep.simxGetObjectFloatParameter(id,armJoints(i),2012,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
            [res,tau(i)] = vrep.simxGetJointForce(id,armJoints(i),vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        end
        % get external force
        [res,state,fexternal,tauexternal] = vrep.simxReadForceSensor(id,forcesensor,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        
        % transfor external force form force sensor frame to robot base frame
        [res,eulerAngles]=vrep.simxGetObjectOrientation(id,forcesensor,-1,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        forcesensorrotation = rotx(double(eulerAngles(1)*180/pi))*roty(double(eulerAngles(2)*180/pi))*rotz(double(eulerAngles(3)*180/pi));
        fexternal_inertial = forcesensorrotation*[fexternal(1);fexternal(2);fexternal(3)];
        
        % shift raw external forces array
        for i=1:windowSize-1
            raw_externalforces(:,i) = raw_externalforces(:,i+1);
        end
        raw_externalforces(1,windowSize) = fexternal_inertial(1);
        raw_externalforces(2,windowSize) = fexternal_inertial(3);
        % force filtering
        filtered_externalforces_x = filter(b,a,raw_externalforces(1,:));
        filtered_externalforces_y = filter(b,a,raw_externalforces(2,:));
        filtered_externalforce = [filtered_externalforces_x(windowSize);filtered_externalforces_y(windowSize)];

        % feedbacked joint force in v-rep is the force sufferred by joint,
        % so change the sign to get the joint drive torque
        tau = -tau;
         
        if (current_time > maxsimtime)
            break;
        end
        
        % get reference trajectory point
        [eeposref,eevref,eearef] = ReferenceTrajectory(current_time,trajectory_mode);
        
       %% impedance control
        if control_mode == 0
            J =Geometric_Jacobian(x0(1:2),[l1;l2]);
            dJ_dt = Geometric_Jacobian_Derivative(x0,[l1;l2]);
            qdotdotref = J\(eearef - dJ_dt*x0(3:4));
            Mass = Mass_Matrix(x0(1:2),[I1 I2 l1 l2 m1 m2 a1 a2 g]');
            u = Impedance_Controller(x0,[l1;l2],[kpx;kpy;kdx;kdy],eeposref,eevref)...
                + Mass*qdotdotref...
                + Coriolis_Centrifugal_Torque(x0,[l1;l2;m1;m2;a1;a2;g])...
                + Gravity_Torque(x0(1:2),[l1;l2;m1;m2;a1;a2;g]);
            % limit joint tourque
            for i=1:2
                if u(i) > 25
                    u(i) = 25;
                elseif u(i) < -25
                    u(i) = -25;
                end
            end
            % send torque command to vrep
            for i=1:2
                if u(i) < 0
                    res = vrep.simxSetJointTargetVelocity(id,armJoints(i),-99999,vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
                    res = vrep.simxSetJointForce(id,armJoints(i),-u(i),vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
                else
                    res = vrep.simxSetJointTargetVelocity(id,armJoints(i),99999,vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
                    res = vrep.simxSetJointForce(id,armJoints(i),u(i),vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
                end
            end
       %% admittance control
        elseif control_mode == 1
            % --- velocity command --- (qdot_d + qdot_admittance)
            % compute qdotdot corresponds to impedance model
            qdotdot_admittance =  Admittance_Controller(x0,filtered_externalforce,[I1 I2 l1 l2 m1 m2 a1 a2 g]',[mx;my;kpx;kpy;kdx;kdy],eeposref,eevref);
            J =Geometric_Jacobian(x0(1:2),[l1;l2]);
            dJ_dt = Geometric_Jacobian_Derivative(x0,[l1;l2]);
            qdotdot_tracking =  J\(eearef - dJ_dt*x0(3:4));
            qdot_admittance = qdot_admittance + (qdotdot_admittance)*dt;
            qdotref  = J\eevref + qdot_admittance;            
            u = qdotref;
            % send velocity command to vrep
            for i=1:2
                vrep.simxSetJointTargetVelocity(id,armJoints(i),qdotref(i),vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
            end
        elseif control_mode == 2
            % --- position command ---  qd + q_admittance
            %compute qdotdot corresponds to impedance model
            qdotdot_admittance =  Admittance_Controller(x0,filtered_externalforce,[I1 I2 l1 l2 m1 m2 a1 a2 g]',[mx;my;kpx;kpy;kdx;kdy],eeposref-eevref*dt,eevref-eearef*dt);%-eevref*dt -eearef*dt
            J =Geometric_Jacobian(x0(1:2),[l1;l2]);
            dJ_dt = Geometric_Jacobian_Derivative(x0,[l1;l2]);
            qdotdot_tracking =  J\(eearef - dJ_dt*x0(3:4));           
            q_admittance = q_admittance + qdot_admittance*dt +  1/2*qdotdot_admittance*dt^2;           
            qdot_admittance = qdot_admittance + qdotdot_admittance*dt;
            qref = Inverse_Kinematics(x0(1:2),eeposref,[l1 l2]) + q_admittance;
            u = qref;
            % send position command to vrep
            for i=1:2
                vrep.simxSetJointTargetPosition(id,armJoints(i),u(i),vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
            end
        end
       
        %% record data
        recordData.x = [recordData.x,x0];
        recordData.tau = [recordData.tau,tau];
        recordData.fmea = [recordData.fmea,raw_externalforces(:,end)];
        recordData.u = [recordData.u,u];
        recordData.t = [recordData.t,current_time];
        
        % update current_time
        current_time = current_time + dt;
        vrep.simxSynchronousTrigger(id);
    end
    % display result
    titles = {'q1','q2','dq1','dq2'};
    figure('name','state');
    for i=1:4
        subplot(2,2,i);
        plot(recordData.t,recordData.x(i,:));
        title(titles{i});
        xlabel('t/s');
    end
    
    titles = {'u1','u2'};
    figure('name','input');
    for i=1:2
        subplot(1,2,i);
        plot(recordData.t,recordData.u(i,:));
        title(titles{i});
        xlabel('t/s');
    end
    
    figure('name','external force');
    subplot(1,2,1);
    plot(recordData.t,recordData.fmea(1,:));
    title('fextx');
    xlabel('t/s');
    hold off
    
    subplot(1,2,2);
    plot(recordData.t,recordData.fmea(2,:));
    title('fexty');
    xlabel('t/s');
    hold off
else
    disp('Failed connecting to remote API server1');
end
vrep.simxStopSimulation(id,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(id);
vrep.delete();
