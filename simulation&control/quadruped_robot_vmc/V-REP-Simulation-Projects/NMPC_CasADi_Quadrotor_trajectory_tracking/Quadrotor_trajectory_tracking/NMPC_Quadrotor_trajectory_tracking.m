%% NMPC based Quadrotor trajectory tracking
clear all
close all
clc
%% define NMPC problem
NMPC_problem_formulation;
load NMPC_problem_definition.mat
%% control simulation

% connect to vrep
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
    % trigger one step simulation
%     vrep.simxSynchronousTrigger(id);
    handles = Quadrotor_workcell_init(vrep,id);
    Quadrotor = handles.Quadrotor;
    pose0 = zeros(6,1);
    velocity0 = zeros(6,1);
    [res,pose0(1:3,1)] = vrep.simxGetObjectPosition(id,Quadrotor,-1,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
    [res,pose0(4:6,1)] = vrep.simxGetObjectOrientation(id,Quadrotor,-1,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
    [res,velocity0(1:3,1),velocity0(4:6,1)] = vrep.simxGetObjectVelocity(id,Quadrotor,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
    t0 = 0;
    % init state
%     x0 = [0;0.25;4;0;0;0;0;0;0;0;0;0];
    x0 = [pose0;velocity0];
    % record states
    xx(:,1) = x0;
    t(1) = t0;
    u0 = zeros(N,length_control);
    X0 = repmat(x0,1,N+1)';
    sim_tim = 20; % max simulation time
    % start NMPC
    nmpciter = 0;
    xx1 = [];
    u_c1= [];
    main_loop  = tic;
    while(vrep.simxGetConnectionId(id)~=-1 && nmpciter<sim_tim/dt)
        current_time = nmpciter*dt;
        % init state
        [res,pose0(1:3,1)] = vrep.simxGetObjectPosition(id,Quadrotor,-1,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        [res,pose0(4:6,1)] = vrep.simxGetObjectOrientation(id,Quadrotor,-1,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        [res,velocity0(1:3,1),velocity0(4:6,1)] = vrep.simxGetObjectVelocity(id,Quadrotor,vrep.simx_opmode_buffer);vrchk(vrep, res, true);
        x0 = [pose0;velocity0];
        args.p(1:length_state,1) = x0;
        % reference
        t_predict = current_time:T:current_time+T*(N-1);
        referencestate = QuadrotorReferenceTrajectory(t_predict);
        referencecontrol = 4.9*ones(4,N);
        reference = [referencestate;referencecontrol];
        args.p(length_state_control-length_control+1:length_state_control*N+length_state,1) = reshape(reference,length_state_control*N,1);
        % init value of OPT variables
        args.x0 = [reshape(X0',length_state*(N+1),1);reshape(u0',length_control*N,1)];
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        xx1(:,1:length_state,nmpciter+1) = reshape(full(sol.x(1:length_state*(N+1)))',length_state,N+1)';
        u = reshape(full(sol.x(length_state*(N+1)+1:end))',length_control,N)';
        u_c1 = [u_c1;u(1,:)];
        t(nmpciter+1) = t0;
        % apply control , integration and  shift the solutionn
        uOptimal = u(1,:);
        for i=1:4
          vrep.simxSetFloatSignal(id,['u',num2str(i)],uOptimal(1,i),vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
        end
        [t0, x0, u0] = shift(dt, t0, x0, u,f);
        xx(:,nmpciter+2) = x0;
        X0 = reshape(full(sol.x(1:length_state*(N+1)))',length_state,N+1)'; % get solution TRAJECTORY
        % Shift trajectory to initialize the next step
        X0 = [X0(2:end,:);X0(end,:)];
        nmpciter
        nmpciter = nmpciter + 1;
        vrep.simxSynchronousTrigger(id);
    end
    main_loop_time = toc(main_loop);
    average_mpc_time = main_loop_time/(nmpciter+1)
    %% display results
    reference = QuadrotorReferenceTrajectory(t);
    % write reference position to file
    % referencexyz = reference(1:3,:)';
    % writematrix(referencexyz,'referencepath.csv');
    titles = {'x','y','z','\phi','\theta','\psi'};
    figure('name','state');
    for i=1:6
        subplot(2,3,i);
        plot(t,xx(i,1:length(t)),t,reference(i,:));
        title(titles{i});
    end
    
    figure('name','input');
    for i=1:4
        subplot(2,2,i);
        plot(t,u_c1(:,i)',t,4.9*ones(1,length(t)));
        title(['u',num2str(i)]);
    end
else
    disp('Failed connecting to remote API server1');
end
vrep.simxStopSimulation(id,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(id);
vrep.delete();