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
sizes.NumInputs      = 9;
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




%旋转矩阵
R={[cos(q1_d) 0  sin(q1_d) ; sin(q1_d)  0 -cos(q1_d); 0 1 0],...
   [cos(q2_d) -sin(q2_d) 0 ; sin(q2_d) cos(q2_d) 0  ; 0 0 1],... 
   [cos(q3_d) -sin(q3_d) 0 ; sin(q3_d) cos(q3_d) 0  ; 0 0 1],... 
   };



%%%%%%%%%%%%%%%%%%%%%%%%%%%
%*******惯性参数*********
%%%%%%%%%%%%%%%%%%%%%%%%%%%

I1= [0.1 0 0 ;0 0.1 0;0 0 0.1];
m1=0.0; l1=0; rc1=[0 0 0]';

I2= [0 0 0 ;0 0.022 0;0 0 0.022];
m2=0.6; l2=0.13; rc2=[0,0,0]';

I3= [0 0 0 ;0 0.0013 0;0 0 0.0013];
m3=0.1; l3=0.13; rc3=[0,0,0]';
p1sta=[0,0,0]';p2sta=[l2,0,0]';p3sta=[l3,0,0]';


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
n11=Ic1*epsilon11+cross(omega11,(Ic1*omega11))+R{2}*n22+cross(rc1,(m1*(ac11-g1)))+cross(p1sta,f11);

tao33_g=(transpose(R{3})*[0,0,1]')'*n33;
tao22_g=(transpose(R{2})*[0,0,1]')'*n22;
tao11_g=(transpose(R{1})*[0,0,1]')'*n11;








sys(1)=tao11_g;
sys(2)=tao22_g;
sys(3)=tao33_g;
sys(4)=0;
sys(5)=0;
sys(6)=0;

