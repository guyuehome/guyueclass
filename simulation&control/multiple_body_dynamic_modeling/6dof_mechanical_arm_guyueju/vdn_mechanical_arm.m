function dstate = vdn_mechanical_arm(state,u,t,i)
global robotarm
% -------------------------------------------------------------------------
% 将输入的状态变量分解成每个杆件的位置与速度
po_arm1     = state(1:6,1);%第一个杆的位置（xyz坐标，欧拉角）
ve_arm1     = state(7:12,1);%第一个杆的速度（xyz坐标一阶导，欧拉角一阶导）
po_arm2     = state(13:18,1);
ve_arm2     = state(19:24,1);
po_arm3     = state(25:30,1);
ve_arm3     = state(31:36,1);
po_arm4     = state(37:42,1);
ve_arm4     = state(43:48,1);
po_arm5     = state(49:54,1);
ve_arm5     = state(55:60,1);
po_arm6     = state(61:66,1);
ve_arm6     = state(67:72,1);
% -------------------------------------------------------------------------
% 旋转矩阵
A_arm1  = [cos(po_arm1(4,1)),0,sin(po_arm1(4,1));0,1,0;-sin(po_arm1(4,1)),0,cos(po_arm1(4,1))]*...
    [cos(po_arm1(5,1)),-sin(po_arm1(5,1)),0;sin(po_arm1(5,1)),cos(po_arm1(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm1(6,1)),-sin(po_arm1(6,1));0,sin(po_arm1(6,1)),cos(po_arm1(6,1))];
A_arm2  = [cos(po_arm2(4,1)),0,sin(po_arm2(4,1));0,1,0;-sin(po_arm2(4,1)),0,cos(po_arm2(4,1))]*...
    [cos(po_arm2(5,1)),-sin(po_arm2(5,1)),0;sin(po_arm2(5,1)),cos(po_arm2(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm2(6,1)),-sin(po_arm2(6,1));0,sin(po_arm2(6,1)),cos(po_arm2(6,1))];
A_arm3  = [cos(po_arm3(4,1)),0,sin(po_arm3(4,1));0,1,0;-sin(po_arm3(4,1)),0,cos(po_arm3(4,1))]*...
    [cos(po_arm3(5,1)),-sin(po_arm3(5,1)),0;sin(po_arm3(5,1)),cos(po_arm3(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm3(6,1)),-sin(po_arm3(6,1));0,sin(po_arm3(6,1)),cos(po_arm3(6,1))];
A_arm4  = [cos(po_arm4(4,1)),0,sin(po_arm4(4,1));0,1,0;-sin(po_arm4(4,1)),0,cos(po_arm4(4,1))]*...
    [cos(po_arm4(5,1)),-sin(po_arm4(5,1)),0;sin(po_arm4(5,1)),cos(po_arm4(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm4(6,1)),-sin(po_arm4(6,1));0,sin(po_arm4(6,1)),cos(po_arm4(6,1))];
A_arm5  = [cos(po_arm5(4,1)),0,sin(po_arm5(4,1));0,1,0;-sin(po_arm5(4,1)),0,cos(po_arm5(4,1))]*...
    [cos(po_arm5(5,1)),-sin(po_arm5(5,1)),0;sin(po_arm5(5,1)),cos(po_arm5(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm5(6,1)),-sin(po_arm5(6,1));0,sin(po_arm5(6,1)),cos(po_arm5(6,1))];
A_arm6  = [cos(po_arm6(4,1)),0,sin(po_arm6(4,1));0,1,0;-sin(po_arm6(4,1)),0,cos(po_arm6(4,1))]*...
    [cos(po_arm6(5,1)),-sin(po_arm6(5,1)),0;sin(po_arm6(5,1)),cos(po_arm6(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm6(6,1)),-sin(po_arm6(6,1));0,sin(po_arm6(6,1)),cos(po_arm6(6,1))];
% 角速度关系矩阵
G_arm1  = [sin(po_arm1(5,1)),0,1;cos(po_arm1(6,1))*cos(po_arm1(5,1)),sin(po_arm1(6,1)),0;-sin(po_arm1(6,1))*cos(po_arm1(5,1)),cos(po_arm1(6,1)),0];
G_arm2  = [sin(po_arm2(5,1)),0,1;cos(po_arm2(6,1))*cos(po_arm2(5,1)),sin(po_arm2(6,1)),0;-sin(po_arm2(6,1))*cos(po_arm2(5,1)),cos(po_arm2(6,1)),0];
G_arm3  = [sin(po_arm3(5,1)),0,1;cos(po_arm3(6,1))*cos(po_arm3(5,1)),sin(po_arm3(6,1)),0;-sin(po_arm3(6,1))*cos(po_arm3(5,1)),cos(po_arm3(6,1)),0];
G_arm4  = [sin(po_arm4(5,1)),0,1;cos(po_arm4(6,1))*cos(po_arm4(5,1)),sin(po_arm4(6,1)),0;-sin(po_arm4(6,1))*cos(po_arm4(5,1)),cos(po_arm4(6,1)),0];
G_arm5  = [sin(po_arm5(5,1)),0,1;cos(po_arm5(6,1))*cos(po_arm5(5,1)),sin(po_arm5(6,1)),0;-sin(po_arm5(6,1))*cos(po_arm5(5,1)),cos(po_arm5(6,1)),0];
G_arm6  = [sin(po_arm6(5,1)),0,1;cos(po_arm6(6,1))*cos(po_arm6(5,1)),sin(po_arm6(6,1)),0;-sin(po_arm6(6,1))*cos(po_arm6(5,1)),cos(po_arm6(6,1)),0];
% 角速度关系矩阵的逆
inv_G_arm1  = [0,cos(po_arm1(6,1))/cos(po_arm1(5,1)),-sin(po_arm1(6,1))/cos(po_arm1(5,1));0,sin(po_arm1(6,1)),cos(po_arm1(6,1));
                1,-sin(po_arm1(5,1))*cos(po_arm1(6,1))/cos(po_arm1(5,1)),sin(po_arm1(5,1))*sin(po_arm1(6,1))/cos(po_arm1(5,1))];
inv_G_arm2  = [0,cos(po_arm2(6,1))/cos(po_arm2(5,1)),-sin(po_arm2(6,1))/cos(po_arm2(5,1));0,sin(po_arm2(6,1)),cos(po_arm2(6,1));
                1,-sin(po_arm2(5,1))*cos(po_arm2(6,1))/cos(po_arm2(5,1)),sin(po_arm2(5,1))*sin(po_arm2(6,1))/cos(po_arm2(5,1))];
inv_G_arm3  = [0,cos(po_arm3(6,1))/cos(po_arm3(5,1)),-sin(po_arm3(6,1))/cos(po_arm3(5,1));0,sin(po_arm3(6,1)),cos(po_arm3(6,1));
                1,-sin(po_arm3(5,1))*cos(po_arm3(6,1))/cos(po_arm3(5,1)),sin(po_arm3(5,1))*sin(po_arm3(6,1))/cos(po_arm3(5,1))];
inv_G_arm4  = [0,cos(po_arm4(6,1))/cos(po_arm4(5,1)),-sin(po_arm4(6,1))/cos(po_arm4(5,1));0,sin(po_arm4(6,1)),cos(po_arm4(6,1));
                1,-sin(po_arm4(5,1))*cos(po_arm4(6,1))/cos(po_arm4(5,1)),sin(po_arm4(5,1))*sin(po_arm4(6,1))/cos(po_arm4(5,1))];
inv_G_arm5  = [0,cos(po_arm5(6,1))/cos(po_arm5(5,1)),-sin(po_arm5(6,1))/cos(po_arm5(5,1));0,sin(po_arm5(6,1)),cos(po_arm5(6,1));
                1,-sin(po_arm5(5,1))*cos(po_arm5(6,1))/cos(po_arm5(5,1)),sin(po_arm5(5,1))*sin(po_arm5(6,1))/cos(po_arm5(5,1))];
inv_G_arm6  = [0,cos(po_arm6(6,1))/cos(po_arm6(5,1)),-sin(po_arm6(6,1))/cos(po_arm6(5,1));0,sin(po_arm6(6,1)),cos(po_arm6(6,1));
                1,-sin(po_arm6(5,1))*cos(po_arm6(6,1))/cos(po_arm6(5,1)),sin(po_arm6(5,1))*sin(po_arm6(6,1))/cos(po_arm6(5,1))];
% 角速度
w_arm1  = G_arm1*ve_arm1(4:6,1);
w_arm2  = G_arm2*ve_arm2(4:6,1);
w_arm3  = G_arm3*ve_arm3(4:6,1);
w_arm4  = G_arm4*ve_arm4(4:6,1);
w_arm5  = G_arm5*ve_arm5(4:6,1);
w_arm6  = G_arm6*ve_arm6(4:6,1);
% 旋转矩阵的一阶导
dA_arm1     = A_arm1*skew(w_arm1);
dA_arm2     = A_arm2*skew(w_arm2);
dA_arm3     = A_arm3*skew(w_arm3);
dA_arm4     = A_arm4*skew(w_arm4);
dA_arm5     = A_arm5*skew(w_arm5);
dA_arm6     = A_arm6*skew(w_arm6);
% 角速度关系矩阵一阶导
dG_arm1  = [ve_arm1(5,1)*cos(po_arm1(5,1)),0,0;
            -ve_arm1(6,1)*sin(po_arm1(6,1))*cos(po_arm1(5,1))-ve_arm1(5,1)*cos(po_arm1(6,1))*sin(po_arm1(5,1)),ve_arm1(6,1)*cos(po_arm1(6,1)),0;
            -ve_arm1(6,1)*cos(po_arm1(6,1))*cos(po_arm1(5,1))+ve_arm1(5,1)*sin(po_arm1(6,1))*sin(po_arm1(5,1)),-ve_arm1(6,1)*sin(po_arm1(6,1)),0];
dG_arm2  = [ve_arm2(5,1)*cos(po_arm2(5,1)),0,0;
            -ve_arm2(6,1)*sin(po_arm2(6,1))*cos(po_arm2(5,1))-ve_arm2(5,1)*cos(po_arm2(6,1))*sin(po_arm2(5,1)),ve_arm2(6,1)*cos(po_arm2(6,1)),0;
            -ve_arm2(6,1)*cos(po_arm2(6,1))*cos(po_arm2(5,1))+ve_arm2(5,1)*sin(po_arm2(6,1))*sin(po_arm2(5,1)),-ve_arm2(6,1)*sin(po_arm2(6,1)),0];
dG_arm3  = [ve_arm3(5,1)*cos(po_arm3(5,1)),0,0;
            -ve_arm3(6,1)*sin(po_arm3(6,1))*cos(po_arm3(5,1))-ve_arm3(5,1)*cos(po_arm3(6,1))*sin(po_arm3(5,1)),ve_arm3(6,1)*cos(po_arm3(6,1)),0;
            -ve_arm3(6,1)*cos(po_arm3(6,1))*cos(po_arm3(5,1))+ve_arm3(5,1)*sin(po_arm3(6,1))*sin(po_arm3(5,1)),-ve_arm3(6,1)*sin(po_arm3(6,1)),0];
dG_arm4  = [ve_arm4(5,1)*cos(po_arm4(5,1)),0,0;
            -ve_arm4(6,1)*sin(po_arm4(6,1))*cos(po_arm4(5,1))-ve_arm4(5,1)*cos(po_arm4(6,1))*sin(po_arm4(5,1)),ve_arm4(6,1)*cos(po_arm4(6,1)),0;
            -ve_arm4(6,1)*cos(po_arm4(6,1))*cos(po_arm4(5,1))+ve_arm4(5,1)*sin(po_arm4(6,1))*sin(po_arm4(5,1)),-ve_arm4(6,1)*sin(po_arm4(6,1)),0];
dG_arm5  = [ve_arm5(5,1)*cos(po_arm5(5,1)),0,0;
            -ve_arm5(6,1)*sin(po_arm5(6,1))*cos(po_arm5(5,1))-ve_arm5(5,1)*cos(po_arm5(6,1))*sin(po_arm5(5,1)),ve_arm5(6,1)*cos(po_arm5(6,1)),0;
            -ve_arm5(6,1)*cos(po_arm5(6,1))*cos(po_arm5(5,1))+ve_arm5(5,1)*sin(po_arm5(6,1))*sin(po_arm5(5,1)),-ve_arm5(6,1)*sin(po_arm5(6,1)),0];
dG_arm6  = [ve_arm6(5,1)*cos(po_arm6(5,1)),0,0;
            -ve_arm6(6,1)*sin(po_arm6(6,1))*cos(po_arm6(5,1))-ve_arm6(5,1)*cos(po_arm6(6,1))*sin(po_arm6(5,1)),ve_arm6(6,1)*cos(po_arm6(6,1)),0;
            -ve_arm6(6,1)*cos(po_arm6(6,1))*cos(po_arm6(5,1))+ve_arm6(5,1)*sin(po_arm6(6,1))*sin(po_arm6(5,1)),-ve_arm6(6,1)*sin(po_arm6(6,1)),0];
% -------------------------------------------------------------------------
% 计算左侧的系数矩阵
robotarm.I1         = [1;0;0];
robotarm.I2         = [0;1;0];
robotarm.I3         = [0;0;1];

Cqa0_1   = calculate_Cqa_one(eye(3,3),A_arm1,[0;0;0],robotarm.u11,[1;0;0],[0;1;0],[0;0;1]);
robotarm.Cqa(1:5,1:6)       = Cqa0_1(:,7:12);
robotarm.Cqa(6:10,1:12)     = calculate_Cqa_one(A_arm1,A_arm2,robotarm.u12,robotarm.u21,robotarm.I1,robotarm.I3,robotarm.I2);
robotarm.Cqa(11:15,7:18)    = calculate_Cqa_one(A_arm2,A_arm3,robotarm.u22,robotarm.u31,robotarm.I1,robotarm.I3,robotarm.I2);
robotarm.Cqa(16:20,13:24)   = calculate_Cqa_one(A_arm3,A_arm4,robotarm.u32,robotarm.u41,robotarm.I1,robotarm.I2,robotarm.I1);
robotarm.Cqa(21:25,19:30)   = calculate_Cqa_one(A_arm4,A_arm5,robotarm.u42,robotarm.u51,robotarm.I1,robotarm.I3,robotarm.I2);
robotarm.Cqa(26:30,25:36)   = calculate_Cqa_one(A_arm5,A_arm6,robotarm.u52,robotarm.u61,robotarm.I2,robotarm.I3,robotarm.I1);
robotarm.equ_left(1:(6*robotarm.n),(6*robotarm.n+1):(11*robotarm.n))    = robotarm.Cqa';
robotarm.equ_left((6*robotarm.n+1):(11*robotarm.n),1:(6*robotarm.n))    = robotarm.Cqa;
% 计算右侧的矩阵
% 计算Qe
Qe_arm1     = [-robotarm.m1*robotarm.g;u(1:3,1)-[0;0;0.2*w_arm1(3,1)]-A_arm1'*A_arm2*(u(4:6,1)+[0;0.2*w_arm2(2,1);0])];
Qe_arm2     = [-robotarm.m2*robotarm.g;u(4:6,1)-[0;0.2*w_arm2(2,1);0]-A_arm2'*A_arm3*(u(7:9,1)+[0;0.2*(w_arm3(2,1)-w_arm2(2,1));0])];
Qe_arm3     = [-robotarm.m3*robotarm.g;u(7:9,1)-[0;0.2*(w_arm3(2,1)-w_arm2(2,1));0]-A_arm3'*A_arm4*(u(10:12,1)+[0.2*w_arm4(1,1);0;0])];
Qe_arm4     = [-robotarm.m4*robotarm.g;u(10:12,1)-[0.2*w_arm4(1,1);0;0]-A_arm4'*A_arm5*(u(13:15,1)+[0;0.2*(w_arm5(2,1)-w_arm4(2,1));0])];
Qe_arm5     = [-robotarm.m5*robotarm.g;u(13:15,1)-[0;0.2*(w_arm5(2,1)-w_arm4(2,1));0]-A_arm5'*A_arm6*(u(16:18,1)+[0.2*w_arm6(1,1);0;0])];
Qe_arm6     = [-robotarm.m6*robotarm.g;u(16:18,1)-[0.2*w_arm6(1,1);0;0]];
Qe          = [Qe_arm1;Qe_arm2;Qe_arm3;Qe_arm4;Qe_arm5;Qe_arm6];
% 计算Qv
Qv          = [0;0;0;-cross(w_arm1,(robotarm.j1*w_arm1));
                0;0;0;-cross(w_arm2,(robotarm.j2*w_arm2));
                0;0;0;-cross(w_arm3,(robotarm.j3*w_arm3));
                0;0;0;-cross(w_arm4,(robotarm.j4*w_arm4));
                0;0;0;-cross(w_arm5,(robotarm.j5*w_arm5));
                0;0;0;-cross(w_arm6,(robotarm.j6*w_arm6))];
% 计算Qd
dCqa0_1     = calculate_dCqa_one(eye(3,3),[0,0,0;0,0,0;0,0,0],A_arm1,dA_arm1,[0;0;0],robotarm.u11,[1;0;0],[0;1;0],[0;0;1]);
robotarm.dCqa(1:5,1:6)      = dCqa0_1(:,7:12);
robotarm.dCqa(6:10,1:12)    = calculate_dCqa_one(A_arm1,dA_arm1,A_arm2,dA_arm2,robotarm.u12,robotarm.u21,robotarm.I1,robotarm.I3,robotarm.I2);
robotarm.dCqa(11:15,7:18)   = calculate_dCqa_one(A_arm2,dA_arm2,A_arm3,dA_arm3,robotarm.u22,robotarm.u31,robotarm.I1,robotarm.I3,robotarm.I2);
robotarm.dCqa(16:20,13:24)  = calculate_dCqa_one(A_arm3,dA_arm3,A_arm4,dA_arm4,robotarm.u32,robotarm.u41,robotarm.I1,robotarm.I2,robotarm.I1);
robotarm.dCqa(21:25,19:30)  = calculate_dCqa_one(A_arm4,dA_arm4,A_arm5,dA_arm5,robotarm.u42,robotarm.u51,robotarm.I1,robotarm.I3,robotarm.I2);
robotarm.dCqa(26:30,25:36)  = calculate_dCqa_one(A_arm5,dA_arm5,A_arm6,dA_arm6,robotarm.u52,robotarm.u61,robotarm.I2,robotarm.I3,robotarm.I1);
dqa         = [ve_arm1(1:3,1);w_arm1;ve_arm2(1:3,1);w_arm2;ve_arm3(1:3,1);w_arm3;ve_arm4(1:3,1);w_arm4;ve_arm5(1:3,1);w_arm5;ve_arm6(1:3,1);w_arm6];
Qd          = -robotarm.dCqa*dqa;

robotarm.equ_right(1:(6*robotarm.n),1)      = Qe+Qv;
robotarm.equ_right((6*robotarm.n+1):end,1)  = Qd;
ddqa        = robotarm.equ_left\robotarm.equ_right;
% -------------------------------------------------------------------------
% 将加速度转换成欧拉角二阶导
dve_arm1    = [ddqa(1:3,1);inv_G_arm1*(ddqa(4:6,1)-dG_arm1*ve_arm1(4:6,1))];
dve_arm2    = [ddqa(7:9,1);inv_G_arm2*(ddqa(10:12,1)-dG_arm2*ve_arm2(4:6,1))];
dve_arm3    = [ddqa(13:15,1);inv_G_arm3*(ddqa(16:18,1)-dG_arm3*ve_arm3(4:6,1))];
dve_arm4    = [ddqa(19:21,1);inv_G_arm4*(ddqa(22:24,1)-dG_arm4*ve_arm4(4:6,1))];
dve_arm5    = [ddqa(25:27,1);inv_G_arm5*(ddqa(28:30,1)-dG_arm5*ve_arm5(4:6,1))];
dve_arm6    = [ddqa(31:33,1);inv_G_arm6*(ddqa(34:36,1)-dG_arm6*ve_arm6(4:6,1))];

dstate      = [ve_arm1;dve_arm1;
                ve_arm2;dve_arm2;
                ve_arm3;dve_arm3;
                ve_arm4;dve_arm4;
                ve_arm5;dve_arm5;
                ve_arm6;dve_arm6];
end