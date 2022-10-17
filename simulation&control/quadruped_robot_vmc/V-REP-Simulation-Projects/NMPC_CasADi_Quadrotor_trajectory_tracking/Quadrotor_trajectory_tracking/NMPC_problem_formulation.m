function NMPC_problem_formulation()
%% import CasADi
import casadi.*

%% define symbolic variables for modeling
x = SX.sym('x');y = SX.sym('y');z = SX.sym('z');
phi = SX.sym('phi');theta = SX.sym('theta');psi = SX.sym('psi');
xdot = SX.sym('xdot');ydot = SX.sym('ydot');zdot = SX.sym('zdot');
phidot = SX.sym('phidot');thetadot = SX.sym('thetadot');psidot = SX.sym('psidot');
state = [x,y,z,phi,theta,psi,xdot,ydot,zdot,phidot,thetadot,psidot]';
length_state = length(state);
u1 = SX.sym('u1');u2 = SX.sym('u2');u3 = SX.sym('u3');u4 = SX.sym('u4');
control = [u1,u2,u3,u4]';
length_control = length(control);
length_state_control = length_state + length_control;
%% define dynamics parameters
Ixx = 1.2;Iyy = 1.2;Izz = 2.3;
k = 1;l = 0.25;m = 2;b = 0.2; g = 9.81;
% Ixx = 0.004;Iyy = 0.0049;Izz = 0.0089;
% k = 0.2392;l = 0.13*2^0.5/2;m = 0.52;b = 0.002; g = 9.81;
%% define system dynamics
% transformation matrix for angular velocities from inertial frame to rates
% of euler angles
Rz = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi), 0;
    0, 0, 1];
Ry = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi), cos(phi)];
EulerAngleType = 'XYZ';
if EulerAngleType == 'XYZ'
    R = Rx*Ry*Rz;
    W = [cos(theta)*cos(psi),sin(psi),0;
    -cos(theta)*sin(psi),cos(psi),0;
    sin(theta),0,1];
elseif EulerAngleType == 'ZYX'
    R = Rz*Ry*Rx;
    W = [1, 0, -sin(theta);
     0, cos(phi), cos(theta)*sin(phi);
     0, -sin(phi), cos(theta)*cos(phi)];
end

% Jacobian (relates body frame angular velocities to the rates of euler angles)
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;

% Coriolis forces
dJ_deulerangle_flat = jacobian(J,[phi,theta,psi]');
dJ_dt_flat = dJ_deulerangle_flat*[phidot,thetadot,psidot]';
dJ_dt = reshape(dJ_dt_flat,3,3);

etadot_J = [phidot,thetadot,psidot]*J;
grad_etadot_J = jacobian(etadot_J,[phi,theta,psi]');
C = dJ_dt - 1/2*grad_etadot_J.';
% Torques in the direction of phi, theta, psi
tau_beta = [l*k*(-u2 + u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];
% tau_beta = [l*k*(u2 - u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];
% Total thrust
Thrust = k*(u1+u2+u3+u4);

% Dynamics
rhs = SX.sym('rhs',12,1);
rhs(1) = xdot;
rhs(2) = ydot;
rhs(3) = zdot;
rhs(4) = phidot;
rhs(5) = thetadot;
rhs(6) = psidot;

% Equations for COM configuration
rhs(7:9) = -g*[0;0;1] + R*[0;0;Thrust]/m;

% Euler Lagrange equations for angular dynamics
rhs(10:12) = inv(J)*(tau_beta - C*[phidot; thetadot; psidot]);

% dynamics derivative
% A = jacobian(rhs,state);
% B = jacobian(rhs,control);

% build function object
f = Function('f',{state,control},{rhs});
% dfdx_dfdu = Function('dfdx_dfdu',{state,control},{A,B});
%% test function
% xk uk for test
% xk = [1,1,1,1,1,1,0.1,0.1,0.1,0.1,0.1,0.1]';
% uk = [1,2,3,4]';
% test Function object
% f_casadi_value = f(xk,uk)
% f_matlab_value = QuadrotorStateFcn(xk,uk);
% [A_casadi_value,B_casadi_value] = dfdx_dfdu(xk,uk);
% [A_matlab_value,B_matlab_value] = QuadrotorStateJacobianFcn(xk,uk);
%% generate and test mex files
% opts = struct('main', false,...
%               'mex', true);
% f.generate('f.c',opts);
% dfdx_dfdu.generate('dfdx_dfdu.c',opts);
% mex -output f.mex f.c
% mex -output dfdx_dfdu.mex dfdx_dfdu.c
% test mex files
% f_value = f('f',xk,uk);
% [A_value,B_value] = dfdx_dfdu('dfdx_dfdu',xk,uk);

%% define MPC problem
% sampling period
T = 0.1;
% prediction horizon
N = 20;
% decision variables
U = SX.sym('U',length_control,N);
% parameters(init state + reference states + reference controls)
P = SX.sym('P',length_state+N*(length_state+length_control),1);
% state variables(current state + predicted states)
X = SX.sym('X',length_state,N+1);
% object function
obj = 0;
% constraints
G = [];
% state weighting matrices
Q = zeros(12,12);
Q(1:3,1:3) = 1*eye(3);
Q(4:6,4:6) = 1*eye(3);
% control weighting matrices
R = 0.1*eye(4);
% init state
st = X(:,1);
% init state constraint
G = [G;st-P(1:length_state,1)];
for k = 1:N
    st = X(:,k); st_next = X(:,k+1);
    con = U(:,k);
    obj = obj + (st - P(length_state_control*k-length_control+1:length_state_control*k-length_control+length_state,1))'*Q*(st - P(length_state_control*k-length_control+1:length_state_control*k-length_control+length_state,1))...
        +(con - P(length_state_control*k-length_control+length_state+1:length_state_control*k+length_state,1))'*R*(con - P(length_state_control*k-length_control+length_state+1:length_state_control*k+length_state,1));
    % dynamics constraints by euler integration
        f_value = f(st,con);
        st_next_euler = st + (T*f_value);
        G = [G;st_next - st_next_euler];
    % dynamics constraints by RK4 integration
%     f_value1 = f(st,con);
%     f_value2 = f(st+T/2*f_value1,con);
%     f_value3 = f(st+T/2*f_value2,con);
%     f_value4 = f(st+T*f_value3,con);
%     st_next_rk4 = st+ (f_value1+2*f_value2+2*f_value3+f_value4)*T/6;
%     G = [G;st_next - st_next_rk4];
end
% flat states and controls
OPT_variables = [reshape(X,length_state*(N+1),1);reshape(U,length_control*N,1)];
% construct a nlp problem
nlp_prob = struct('f',obj,'x',OPT_variables,'g',G,'p',P);
% options for OCP solver
opts = struct;
opts.ipopt.max_iter = 300;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-6;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
% construct a solver
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
% limits for constraints ststes and controls
args.lbg(1:length_state*(N+1),1) = 0;  % -1e-20  % Equality constraints
args.ubg(1:length_state*(N+1),1) = 0;  % 1e-20   % Equality constraints
args.lbx(1:length_state*(N+1),1) = -inf;
args.ubx(1:length_state*(N+1),1) = inf;
ulower = [0,0,0,0]';
uupper = [12,12,12,12]';
args.lbx(length_state*(N+1)+1:length_control:length_state*(N+1)+length_control*N,1) = ulower(1);
args.lbx(length_state*(N+1)+2:length_control:length_state*(N+1)+length_control*N,1) = ulower(2);
args.lbx(length_state*(N+1)+3:length_control:length_state*(N+1)+length_control*N,1) = ulower(3);
args.lbx(length_state*(N+1)+4:length_control:length_state*(N+1)+length_control*N,1) = ulower(4);
args.ubx(length_state*(N+1)+1:length_control:length_state*(N+1)+length_control*N,1) = uupper(1);
args.ubx(length_state*(N+1)+2:length_control:length_state*(N+1)+length_control*N,1) = uupper(2);
args.ubx(length_state*(N+1)+3:length_control:length_state*(N+1)+length_control*N,1) = uupper(3);
args.ubx(length_state*(N+1)+4:length_control:length_state*(N+1)+length_control*N,1) = uupper(4);
save NMPC_problem_definition.mat length_state length_control length_state_control f T N solver args 
clear;
end