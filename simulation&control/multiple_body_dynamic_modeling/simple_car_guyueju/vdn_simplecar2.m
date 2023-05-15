function dstate = vdn_simplecar2(state,u,t,i)
global car
% -------------------------------------------------------------------------
% 根据输入的状态变量进行初始计算
qi          = state(1:12,1);
dqi         = state(13:24,1);
% 计算每个部件的欧拉角
I1          = [1;0;0];
I2          = [0;1;0];
I3          = [0;0;1];
eu1         = state(4:6,1);
eu2         = eu1+I1*state(7,1);
eu3         = eu2+I3*state(8,1);
eu4         = eu1+I3*state(9,1);
eu5         = eu1+I1*state(10,1);
eu6         = eu5+I3*state(11,1);
eu7         = eu1+I3*state(12,1);
deu1        = state(16:18,1);
deu2        = deu1+I1*state(19,1);
deu3        = deu2+I3*state(20,1);
deu4        = deu1+I3*state(21,1);
deu5        = deu1+I1*state(22,1);
deu6        = deu5+I3*state(23,1);
deu7        = deu1+I3*state(24,1);
% -------------------------------------------------------------------------
% 旋转矩阵和角速度系数矩阵及其一阶导
A1      = [cos(eu1(1,1)),-sin(eu1(1,1)),0;sin(eu1(1,1)),cos(eu1(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu1(2,1)),-sin(eu1(2,1));0,sin(eu1(2,1)),cos(eu1(2,1))]*...
          [cos(eu1(3,1)),0,sin(eu1(3,1));0,1,0;-sin(eu1(3,1)),0,cos(eu1(3,1))];
A2      = [cos(eu2(1,1)),-sin(eu2(1,1)),0;sin(eu2(1,1)),cos(eu2(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu2(2,1)),-sin(eu2(2,1));0,sin(eu2(2,1)),cos(eu2(2,1))]*...
          [cos(eu2(3,1)),0,sin(eu2(3,1));0,1,0;-sin(eu2(3,1)),0,cos(eu2(3,1))];
A3      = [cos(eu3(1,1)),-sin(eu3(1,1)),0;sin(eu3(1,1)),cos(eu3(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu3(2,1)),-sin(eu3(2,1));0,sin(eu3(2,1)),cos(eu3(2,1))]*...
          [cos(eu3(3,1)),0,sin(eu3(3,1));0,1,0;-sin(eu3(3,1)),0,cos(eu3(3,1))];
A4      = [cos(eu4(1,1)),-sin(eu4(1,1)),0;sin(eu4(1,1)),cos(eu4(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu4(2,1)),-sin(eu4(2,1));0,sin(eu4(2,1)),cos(eu4(2,1))]*...
          [cos(eu4(3,1)),0,sin(eu4(3,1));0,1,0;-sin(eu4(3,1)),0,cos(eu4(3,1))];
A5      = [cos(eu5(1,1)),-sin(eu5(1,1)),0;sin(eu5(1,1)),cos(eu5(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu5(2,1)),-sin(eu5(2,1));0,sin(eu5(2,1)),cos(eu5(2,1))]*...
          [cos(eu5(3,1)),0,sin(eu5(3,1));0,1,0;-sin(eu5(3,1)),0,cos(eu5(3,1))];
A6      = [cos(eu6(1,1)),-sin(eu6(1,1)),0;sin(eu6(1,1)),cos(eu6(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu6(2,1)),-sin(eu6(2,1));0,sin(eu6(2,1)),cos(eu6(2,1))]*...
          [cos(eu6(3,1)),0,sin(eu6(3,1));0,1,0;-sin(eu6(3,1)),0,cos(eu6(3,1))];
A7      = [cos(eu7(1,1)),-sin(eu7(1,1)),0;sin(eu7(1,1)),cos(eu7(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu7(2,1)),-sin(eu7(2,1));0,sin(eu7(2,1)),cos(eu7(2,1))]*...
          [cos(eu7(3,1)),0,sin(eu7(3,1));0,1,0;-sin(eu7(3,1)),0,cos(eu7(3,1))];

G1_     = [-sin(eu1(3,1))*cos(eu1(2,1)),cos(eu1(3,1)),0;sin(eu1(2,1)),0,1;cos(eu1(3,1))*cos(eu1(2,1)),sin(eu1(3,1)),0];
G2_     = [-sin(eu2(3,1))*cos(eu2(2,1)),cos(eu2(3,1)),0;sin(eu2(2,1)),0,1;cos(eu2(3,1))*cos(eu2(2,1)),sin(eu2(3,1)),0];
G3_     = [-sin(eu3(3,1))*cos(eu3(2,1)),cos(eu3(3,1)),0;sin(eu3(2,1)),0,1;cos(eu3(3,1))*cos(eu3(2,1)),sin(eu3(3,1)),0];
G4_     = [-sin(eu4(3,1))*cos(eu4(2,1)),cos(eu4(3,1)),0;sin(eu4(2,1)),0,1;cos(eu4(3,1))*cos(eu4(2,1)),sin(eu4(3,1)),0];
G5_     = [-sin(eu5(3,1))*cos(eu5(2,1)),cos(eu5(3,1)),0;sin(eu5(2,1)),0,1;cos(eu5(3,1))*cos(eu5(2,1)),sin(eu5(3,1)),0];
G6_     = [-sin(eu6(3,1))*cos(eu6(2,1)),cos(eu6(3,1)),0;sin(eu6(2,1)),0,1;cos(eu6(3,1))*cos(eu6(2,1)),sin(eu6(3,1)),0];
G7_     = [-sin(eu7(3,1))*cos(eu7(2,1)),cos(eu7(3,1)),0;sin(eu7(2,1)),0,1;cos(eu7(3,1))*cos(eu7(2,1)),sin(eu7(3,1)),0];

w1_     = G1_*deu1;
w2_     = G2_*deu2;
w3_     = G3_*deu3;
w4_     = G4_*deu4;
w5_     = G5_*deu5;
w6_     = G6_*deu6;
w7_     = G7_*deu7;

dA1     = A1*skew(w1_);
dA2     = A2*skew(w2_);
dA3     = A3*skew(w3_);
dA4     = A4*skew(w4_);
dA5     = A5*skew(w5_);
dA6     = A6*skew(w6_);
dA7     = A7*skew(w7_);

dG1_    = [-cos(eu1(3,1))*cos(eu1(2,1))*deu1(3,1)+sin(eu1(3,1))*sin(eu1(2,1))*deu1(2,1),-sin(eu1(3,1))*deu1(3,1),0;
            cos(eu1(2,1))*deu1(2,1)                                                     ,0                      ,0;
            -sin(eu1(3,1))*cos(eu1(2,1))*deu1(3,1)-cos(eu1(3,1))*sin(eu1(2,1))*deu1(2,1),cos(eu1(3,1))*deu1(3,1),0];
dG2_    = [-cos(eu2(3,1))*cos(eu2(2,1))*deu2(3,1)+sin(eu2(3,1))*sin(eu2(2,1))*deu2(2,1),-sin(eu2(3,1))*deu2(3,1),0;
            cos(eu2(2,1))*deu2(2,1)                                                     ,0                      ,0;
            -sin(eu2(3,1))*cos(eu2(2,1))*deu2(3,1)-cos(eu2(3,1))*sin(eu2(2,1))*deu2(2,1),cos(eu2(3,1))*deu2(3,1),0];
dG3_    = [-cos(eu3(3,1))*cos(eu3(2,1))*deu3(3,1)+sin(eu3(3,1))*sin(eu3(2,1))*deu3(2,1),-sin(eu3(3,1))*deu3(3,1),0;
            cos(eu3(2,1))*deu3(2,1)                                                     ,0                      ,0;
            -sin(eu3(3,1))*cos(eu3(2,1))*deu3(3,1)-cos(eu3(3,1))*sin(eu3(2,1))*deu3(2,1),cos(eu3(3,1))*deu3(3,1),0];
dG4_    = [-cos(eu4(3,1))*cos(eu4(2,1))*deu4(3,1)+sin(eu4(3,1))*sin(eu4(2,1))*deu4(2,1),-sin(eu4(3,1))*deu4(3,1),0;
            cos(eu4(2,1))*deu4(2,1)                                                     ,0                      ,0;
            -sin(eu4(3,1))*cos(eu4(2,1))*deu4(3,1)-cos(eu4(3,1))*sin(eu4(2,1))*deu4(2,1),cos(eu4(3,1))*deu4(3,1),0];
dG5_    = [-cos(eu5(3,1))*cos(eu5(2,1))*deu5(3,1)+sin(eu5(3,1))*sin(eu5(2,1))*deu5(2,1),-sin(eu5(3,1))*deu5(3,1),0;
            cos(eu5(2,1))*deu5(2,1)                                                     ,0                      ,0;
            -sin(eu5(3,1))*cos(eu5(2,1))*deu5(3,1)-cos(eu5(3,1))*sin(eu5(2,1))*deu5(2,1),cos(eu5(3,1))*deu5(3,1),0];
dG6_    = [-cos(eu6(3,1))*cos(eu6(2,1))*deu6(3,1)+sin(eu6(3,1))*sin(eu6(2,1))*deu6(2,1),-sin(eu6(3,1))*deu6(3,1),0;
            cos(eu6(2,1))*deu6(2,1)                                                     ,0                      ,0;
            -sin(eu6(3,1))*cos(eu6(2,1))*deu6(3,1)-cos(eu6(3,1))*sin(eu6(2,1))*deu6(2,1),cos(eu6(3,1))*deu6(3,1),0];
dG7_    = [-cos(eu7(3,1))*cos(eu7(2,1))*deu7(3,1)+sin(eu7(3,1))*sin(eu7(2,1))*deu7(2,1),-sin(eu7(3,1))*deu7(3,1),0;
            cos(eu7(2,1))*deu7(2,1)                                                     ,0                      ,0;
            -sin(eu7(3,1))*cos(eu7(2,1))*deu7(3,1)-cos(eu7(3,1))*sin(eu7(2,1))*deu7(2,1),cos(eu7(3,1))*deu7(3,1),0];
% -------------------------------------------------------------------------
% 计算左侧系数矩阵
% 计算矩阵B与dB
I0  = zeros(3,3);
I   = eye(3,3);
B   = [I    ,zeros(3,9);
        I0  ,G1_                                                        ,zeros(3,6);
        I   ,-A1*skew(car.u12)*G1_+A2*skew(car.u21)*G2_                 ,A2*skew(car.u21)*G2_*I1                ,zeros(3,5);
        I0  ,G2_                                                        ,G2_*I1                                 ,zeros(3,5);
        I   ,-A1*skew(car.u12)*G1_+A2*(skew(car.u21)-skew(car.u23))*G2_ ,A2*(skew(car.u21)-skew(car.u23))*G2_*I1,zeros(3,5);
        I0  ,G3_                                                        ,G3_*I1                                 ,I2         ,zeros(3,4);
        I   ,-A1*skew(car.u14)*G1_                                      ,zeros(3,6);
        I0  ,G4_                                                        ,zeros(3,2)                             ,I2         ,I0;

        I   ,-A1*skew(car.u15)*G1_+A5*skew(car.u51)*G5_                 ,I0         ,A5*skew(car.u51)*G5_*I1                ,zeros(3,2);
        I0  ,G5_                                                        ,I0         ,G5_*I1                                 ,zeros(3,2);
        I   ,-A1*skew(car.u15)*G1_+A5*(skew(car.u51)-skew(car.u56))*G5_ ,I0         ,A5*(skew(car.u51)-skew(car.u56))*G5_*I1,zeros(3,2);
        I0  ,G6_                                                        ,I0         ,G6_*I1                                 ,I2         ,zeros(3,1);
        I   ,-A1*skew(car.u17)*G1_                                      ,zeros(3,6) ;
        I0  ,G7_                                                        ,zeros(3,5) ,I2];
dB  = [I0   ,zeros(3,9);
        I0  ,dG1_                                                        ,zeros(3,6);
        I0  ,-dA1*skew(car.u12)*G1_-A1*skew(car.u12)*dG1_+dA2*skew(car.u21)*G2_+A2*skew(car.u21)*dG2_                                   ,dA2*skew(car.u21)*G2_*I1+A2*skew(car.u21)*dG2_*I1                                  ,zeros(3,5);
        I0  ,dG2_                                                                                                                       ,dG2_*I1                                                                            ,zeros(3,5);
        I0  ,-dA1*skew(car.u12)*G1_-A1*skew(car.u12)*dG1_+dA2*(skew(car.u21)-skew(car.u23))*G2_+A2*(skew(car.u21)-skew(car.u23))*dG2_   ,dA2*(skew(car.u21)-skew(car.u23))*G2_*I1+A2*(skew(car.u21)-skew(car.u23))*dG2_*I1  ,zeros(3,5);
        I0  ,dG3_                                                        ,dG3_*I1                                 ,zeros(3,5);
        I0  ,-dA1*skew(car.u14)*G1_-A1*skew(car.u14)*dG1_                ,zeros(3,6);
        I0  ,dG4_                                                        ,zeros(3,6);

        I0  ,-dA1*skew(car.u15)*G1_-A1*skew(car.u15)*dG1_+dA5*skew(car.u51)*G5_+A5*skew(car.u51)*dG5_                                   ,I0         ,dA5*skew(car.u51)*G5_*I1+A5*skew(car.u51)*dG5_*I1                ,zeros(3,2);
        I0  ,dG5_                                                        ,I0         ,dG5_*I1                                 ,zeros(3,2);
        I0  ,-dA1*skew(car.u15)*G1_-A1*skew(car.u15)*dG1_+dA5*(skew(car.u51)-skew(car.u56))*G5_+A5*(skew(car.u51)-skew(car.u56))*dG5_   ,I0         ,dA5*(skew(car.u51)-skew(car.u56))*G5_*I1+A5*(skew(car.u51)-skew(car.u56))*dG5_*I1 ,zeros(3,2);
        I0  ,dG6_                                                        ,I0         ,dG6_*I1                                 ,zeros(3,2);
        I0  ,-dA1*skew(car.u17)*G1_-A1*skew(car.u17)*dG1_                ,zeros(3,6);
        I0  ,dG7_                                                        ,zeros(3,6)];
Mi  = B.'*car.M*B;
% -------------------------------------------------------------------------
% 计算右侧的矩阵
% 计算轮胎垂向力
R3      = qi(1:3,1)+A1*car.u12-A2*car.u21+A2*car.u23;
R4      = qi(1:3,1)+A1*car.u14;
R6      = qi(1:3,1)+A1*car.u15-A5*car.u51+A5*car.u56;
R7      = qi(1:3,1)+A1*car.u17;
dqa     = B*dqi;
z_tire  = [R3(3,1),R4(3,1),R6(3,1),R7(3,1)]-car.r_tire;
dz_tire = [dqa(15,1),dqa(21,1),dqa(33,1),dqa(39,1)];
Fz_tire = (-car.k_t.*z_tire.*(z_tire<0)-car.c_t.*dz_tire.*(z_tire<0).*(dz_tire<0));%1*4
% 摩擦力
dR3_    = A2'*dqa(13:15,1);
dR4_    = A1'*dqa(19:21,1);
dR6_    = A5'*dqa(31:33,1);
dR7_    = A1'*dqa(37:39,1);

v_groud = [[dR3_(1,1)-w3_(2,1)*car.r_tire;dR3_(2,1);0],...
           [dR4_(1,1)-w4_(2,1)*car.r_tire;dR4_(2,1);0],...
           [dR6_(1,1)-w6_(2,1)*car.r_tire;dR6_(2,1);0],...
           [dR7_(1,1)-w7_(2,1)*car.r_tire;dR7_(2,1);0]];
delta_v = sqrt(v_groud(1,:).*v_groud(1,:)+v_groud(2,:).*v_groud(2,:));

logi1   = delta_v<-car.f_v0;
logi2   = (delta_v>=-car.f_v0&delta_v<=car.f_v0);
logi3   = delta_v>car.f_v0;
Ff_     = 0.*delta_v;
Ff_(logi1)  = car.mu0.*Fz_tire(logi1);
Ff_(logi2)  = car.mu0./car.f_v0.*delta_v(logi2).*Fz_tire(logi2);
Ff_(logi3)  = -car.mu0.*Fz_tire(logi3);

delta_v_mod     = delta_v+(delta_v==0);
Ff      = [A2*v_groud(:,1)/delta_v_mod(1,1)*Ff_(1,1),...
           A1*v_groud(:,2)/delta_v_mod(1,2)*Ff_(1,2),...
           A5*v_groud(:,3)/delta_v_mod(1,3)*Ff_(1,3),...
           A1*v_groud(:,4)/delta_v_mod(1,4)*Ff_(1,4)];

Mf3     = A3'*cross([0;0;-car.r_tire],Ff(:,1));
Mf4     = A4'*cross([0;0;-car.r_tire],Ff(:,2));
Mf6     = A6'*cross([0;0;-car.r_tire],Ff(:,3));
Mf7     = A7'*cross([0;0;-car.r_tire],Ff(:,4));
% 计算Qe
Qe1         = [0;0;-car.m_body*car.g;0;0;0];%-u(1,1)-u(2,1)];
Qe2         = [0;0;-car.m_rod*car.g;0;0;u(1,1)];
Qe3         = [0;0;-car.m_tire*car.g+Fz_tire(1,1);0;0;0]+[Ff(:,1);Mf3];
Qe4         = [0;0;-car.m_tire2*car.g+Fz_tire(1,2);0;u(3,1);0]+[Ff(:,2);Mf4];
Qe5         = [0;0;-car.m_rod*car.g;0;0;u(2,1)];
Qe6         = [0;0;-car.m_tire*car.g+Fz_tire(1,3);0;0;0]+[Ff(:,3);Mf6];
Qe7         = [0;0;-car.m_tire2*car.g+Fz_tire(1,4);0;u(4,1);0]+[Ff(:,4);Mf7];
Qe          = [Qe1;Qe2;Qe3;Qe4;Qe5;Qe6;Qe7];
% 计算Qv
Qv          = [0;0;0;-cross(w1_,(car.J_body*w1_));
                0;0;0;-cross(w2_,(car.J_rod*w2_));
                0;0;0;-cross(w3_,(car.J_tire*w3_));
                0;0;0;-cross(w4_,(car.J_tire*w4_));
                0;0;0;-cross(w5_,(car.J_rod*w5_));
                0;0;0;-cross(w6_,(car.J_tire*w6_));
                0;0;0;-cross(w7_,(car.J_tire*w7_))];
% 计算Qi
Qi      = B'*(Qe+Qv-car.M*dB*dqi);
% 求解方程
ddqi    = Mi\Qi;

ddqi(7,1)   = 0;
ddqi(10,1)  = 0;

dstate      = [dqi;ddqi];
end