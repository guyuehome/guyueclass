function main_simplecar
global car 
% model parameter
I           = eye(3,3);
car.m_body  = 1000;
car.m_rod   = 1;
car.m_tire  = 9;
car.m_tire2  = 10;
car.J_body  = [1,0,0;0,1,0;0,0,1];
car.J_rod   = [1,0,0;0,1,0;0,0,1];
car.J_tire  = [1,0,0;0,1,0;0,0,1];
car.M       = blkdiag(car.m_body*I,car.J_body,car.m_rod*I,car.J_rod,car.m_tire*I,car.J_tire,car.m_tire2*I,car.J_tire, ...
                        car.m_rod*I,car.J_rod,car.m_tire*I,car.J_tire,car.m_tire2*I,car.J_tire);
car.body_a  = 2;
car.body_b  = 1;
car.body_c  = 0.1;
car.rod_ly  = 0.1;
car.rod_lz  = 0.1;
car.rod_a   = 0.02;
car.rod_b   = 0.08;
car.rod_c   = 0.04;
car.rod_d   = 0.04;
car.r_tire  = 0.1;

car.u12     = [0.8;-0.5;0];
car.u21     = [0;0;0.1];
car.u23     = [0;-0.1;0];
car.u14     = [-0.8;-0.6;-0.1];
car.u15     = [0.8;0.5;0];
car.u51     = [0;0;0.1];
car.u56     = [0;0.1;0];
car.u17     = [-0.8;0.6;-0.1];

car.g       = 10;%9.8;
car.k_t     = 100000;
car.c_t     = 1000;
car.f_v0    = 0.001;
car.mu0     = 0.7;

car.V_body  = [-car.body_a/2,-car.body_b/2,-car.body_c/2;-car.body_a/2,-car.body_b/2,car.body_c/2;
                -car.body_a/2,car.body_b/2,car.body_c/2;-car.body_a/2,car.body_b/2,-car.body_c/2;
                car.body_a/2,-car.body_b/2,-car.body_c/2;car.body_a/2,-car.body_b/2,car.body_c/2;
                car.body_a/2,car.body_b/2,car.body_c/2;car.body_a/2,car.body_b/2,-car.body_c/2];
car.F_body  = [1,2,3,4;1,5,6,2;1,5,8,4;5,6,7,8;2,6,7,3;7,8,4,3];
car.Vr_rod  = [-car.rod_d/2,car.rod_a,-car.rod_a;-car.rod_d/2,-car.rod_ly,-car.rod_a;
               -car.rod_d/2,-car.rod_ly,car.rod_c-car.rod_a;-car.rod_d/2,-car.rod_ly+car.rod_b,car.rod_c-car.rod_a;
               -car.rod_d/2,-car.rod_ly+car.rod_b,car.rod_lz;-car.rod_d/2,car.rod_a,car.rod_lz;
               car.rod_d/2,car.rod_a,-car.rod_a;car.rod_d/2,-car.rod_ly,-car.rod_a;
               car.rod_d/2,-car.rod_ly,car.rod_c-car.rod_a;car.rod_d/2,-car.rod_ly+car.rod_b,car.rod_c-car.rod_a;
               car.rod_d/2,-car.rod_ly+car.rod_b,car.rod_lz;car.rod_d/2,car.rod_a,car.rod_lz];
car.Vl_rod  = [-car.rod_d/2,-car.rod_a,-car.rod_a;-car.rod_d/2,car.rod_ly,-car.rod_a;
               -car.rod_d/2,car.rod_ly,car.rod_c-car.rod_a;-car.rod_d/2,car.rod_ly-car.rod_b,car.rod_c-car.rod_a;
               -car.rod_d/2,car.rod_ly-car.rod_b,car.rod_lz;-car.rod_d/2,-car.rod_a,car.rod_lz;
               car.rod_d/2,-car.rod_a,-car.rod_a;car.rod_d/2,car.rod_ly,-car.rod_a;
               car.rod_d/2,car.rod_ly,car.rod_c-car.rod_a;car.rod_d/2,car.rod_ly-car.rod_b,car.rod_c-car.rod_a;
               car.rod_d/2,car.rod_ly-car.rod_b,car.rod_lz;car.rod_d/2,-car.rod_a,car.rod_lz;];
car.F_rod   = [1,2,3,4,5,6;2,8,9,10,4,3;4,10,11,12,6,5;6,12,7,8,2,1;7,8,9,10,11,12];
car.x_tire  = [0,car.r_tire*cos([0:(2*pi/20):(2*pi)]),0];
car.y_tire  = zeros(1,20+3);
car.z_tire  = [0,car.r_tire*sin([0:(2*pi/20):(2*pi)]),0];
car.u_tire  = [car.x_tire;car.y_tire;car.z_tire];

% 仿真设置
t_all       = 10;
dt          = 0.005;
N           = t_all/dt;

% initial parameter
% 1-6车身的位置及欧拉角，7是右前轮横摆角，8右前车轮自转角度，9右后轮自转角度，10是左前轮横摆角，11左前车轮自转角度，12左后轮自转角度
% 13-24上述数据的一阶导
Xin         = [1;0;0.2-0.026;0;0;0;0+pi/16;0;0;0+pi/16;0;0;
                0;0;0;0;0;0;0;0;0;0;0;0];
state       = zeros(24,N+1);
state(:,1)  = Xin;

figure(1);
% main
for i = 1:N
    t   = i*dt;
    u   = [0;0;60;60];% 1-2右左前轮的横摆角控制力矩，3-4右左后轮的驱动力矩
    plotrealtime(state(:,i),i,state,t);
    state(:,i+1)    = runge_kutta4(@vdn_simplecar2,state(:,i),u,t,dt,i);
end
end