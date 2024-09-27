function [sys,x0,str,ts] = control(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 15;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [-1 0];
function sys=mdlOutputs(t,x,u)


%期望关节状态
q1_d=u(1);   q2_d=u(2);   q3_d=u(3); 
qd1_d=u(4);  qd2_d=u(5);  qd3_d=u(6); 
qdd1_d=u(7); qdd2_d=u(8); qdd3_d=u(9);

%实际返回的关节状态
q1_b=u(10); qd1_b=u(11);  %返回的关节1
q2_b=u(12); qd2_b=u(13);  %返回的关节2
q3_b=u(14); qd3_b=u(15);  %返回的关节3

%误差
e1=q1_d-q1_b; de1=qd1_d-qd1_b;  %关节1误差
e2=q2_d-q2_b; de2=qd2_d-qd2_b;  %关节2误差
e3=q3_d-q3_b; de3=qd3_d-qd3_b;  %关节2误差

tao_pid_1 = 6*e1 + 0.9*de1;
tao_pid_2 = 6*e2 + 0.9*de2;
tao_pid_3 = 6*e3 + 0.9*de3;



I1= [0.1 0 0 ;0 0.1 0;0 0 0.1];
m1=0; l1=0; rc1=[0,0,0]';

I2= [0 0 0 ;0 0.012 0;0 0 0.012];
m2 = 0.6; l2 = 0.13; rc2=[-0.0,0,0]';

I3= [0 0 0 ;0 0.0013 0;0 0 0.0013];
m3 = 0.1; l3 = 0.13; rc3=[0,0,0]';

p1sta=[0,0,0]';p2sta=[l2,0,0]';p3sta=[l3,0,0]';


%旋转矩阵
R={[cos(q1_b) 0  sin(q1_b) ; sin(q1_b)  0 -cos(q1_b); 0 1 0],...
   [cos(q2_b) -sin(q2_b) 0 ; sin(q2_b) cos(q2_b) 0  ; 0 0 1],... 
   [cos(q3_b) -sin(q3_b) 0 ; sin(q3_b) cos(q3_b) 0  ; 0 0 1],... 
   };


%%%%%%%%%%%%%
%正速度计算
%%%%%%%%%%%%%
omega00=[0,0,0]';
omega11=R{1}'*(omega00+[0,0,1]'*qd1_d);
omega22=R{2}'*(omega11+[0,0,1]'*qd2_d);
omega33=R{3}'*(omega22+[0,0,1]'*qd3_d);
v00=[0,0,0]';
v11=R{1}'*v00+cross(omega11,p1sta);
v22=R{2}'*v11+cross(omega22,p2sta);
v33=R{3}'*v22+cross(omega33,p3sta);
v03=R{1}*R{2}*R{3}*v33;
%正加速度计算
epsilon00=[0,0,0]';
epsilon11=transpose(R{1})*(epsilon00+cross(omega00,[0,0,1]'*qd1_d)+[0,0,1]'*qdd1_d);
epsilon22=transpose(R{2})*(epsilon11+cross(omega11,[0,0,1]'*qd2_d)+[0,0,1]'*qdd2_d);
epsilon33=transpose(R{3})*(epsilon22+cross(omega22,[0,0,1]'*qd3_d)+[0,0,1]'*qdd3_d);
a00=[0,0,0]';
a11=transpose(R{1})*a00+cross(epsilon11,p1sta)+cross(omega11,cross(omega11,p1sta));
a22=transpose(R{2})*a11+cross(epsilon22,p2sta)+cross(omega22,cross(omega22,p2sta));
a33=transpose(R{3})*a22+cross(epsilon33,p3sta)+cross(omega33,cross(omega33,p3sta));
ac11=a11+cross(epsilon11,rc1)+cross(omega11,cross(omega11,rc1));
ac22=a22+cross(epsilon22,rc2)+cross(omega22,cross(omega22,rc2));
ac33=a33+cross(epsilon33,rc3)+cross(omega33,cross(omega33,rc3));
%%计算力


g1=transpose(R{1})*[0,0,-9.81]';
g2=transpose(R{1}*R{2})*[0,0,-9.81]';
g3=transpose(R{1}*R{2}*R{3})*[0,0,-9.81]';

Ic1=I1; Ic2=I2; Ic3=I3;

f44=[0,0,0]'; n44=[0,0,0]';

f33=m3*(ac33-g3)+     f44 ;
f22=m2*(ac22-g2)+R{3}*f33; 
f11=m1*(ac11-g1)+R{2}*f22; 


n33=Ic3*epsilon33+cross(omega33,(Ic3*omega33))+     n44+cross(p3sta,f33)+cross(rc3,(m3*(ac33-g3)));
n22=Ic2*epsilon22+cross(omega22,(Ic2*omega22))+R{3}*n33+cross(p2sta,f22)+cross(rc2,(m2*(ac22-g2)));
n11=Ic1*epsilon11+cross(omega11,(Ic1*omega11))+R{2}*n22+cross(p1sta,f11)+cross(rc1,(m1*(ac11-g1)));

tao33_g=(transpose(R{3})*[0,0,1]')'*n33;
tao22_g=(transpose(R{2})*[0,0,1]')'*n22;
tao11_g=(transpose(R{1})*[0,0,1]')'*n11;

tao1 = tao_pid_1 + 1*tao11_g;
tao2 = tao_pid_2 + 1*tao22_g;
tao3 = tao_pid_3 + 1*tao33_g;




sys(1)=tao1;
sys(2)=tao2;
sys(3)=tao3;
sys(4)=0;
sys(5)=0;
sys(6)=0;

