function [sys,x0,str,ts] = ikinamatic(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 9;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [-1 0];
function sys=mdlOutputs(t,x,u)


l1 = 0; 
l2 = 0.13;
l3 = 0.13;
elbow=-1;
Px=u(1);     Py=u(2);    Pz=u(3);
Pdx=u(4);    Pdy=u(5);   Pdz=u(6);
Pddx=u(7);   Pddy=u(8);  Pddz=u(9);
    
%*求*关*节*3*
if Py<0
    fuY=-1;
else
    fuY=1;
end
r=Px*Px+Pz*Pz+Py*Py;
%%%%%%%%%%%%%%%%%%%%%%%%
q3=elbow*acos(( (r-l2^2-l3^2) / (2*l2*l3) ));
%%%%%%%%%%%%%%%%%%%%%%%%

%*求*关*节*1*%%%%%
q1=atan2(Py,Px);%
%%%%%%%%%%%%%%%%%

%*求*关*节*2***以*X*坐标为等量关系
%没有y轴的正负之别，全部取正号
a=l2*cos(q1) + l3*cos(q1)*cos(q3);
b=-l3*cos(q1)*sin(q3); 
c=Px;
if a==0
    q21=pi/2;
else
    q21=atan(b/a);
end
if c==0
    q22=fuY*pi/2;
else
    q22=atan( (sqrt(a*a+b*b-c*c) ) / (c) );
end
%%%%%%%%%%%%%%
q2=(q21+q22);%
%%%%%%%%%%%%%%


%旋转矩阵
R={[cos(q1) 0 sin(q1)  ; sin(q1)  0 -cos(q1); 0 1 0],...
   [cos(q2) -sin(q2) 0 ; sin(q2) cos(q2) 0  ; 0 0 1],... 
   [cos(q3) -sin(q3) 0 ; sin(q3) cos(q3) 0  ; 0 0 1],... 
   };
%原点坐标
P={[0 0 0]' , [l2*cos(q2) l2*sin(q2) 0]' , [l3*cos(q3) l3*sin(q3) 0]' };


%%%%%%%%%%%%%%%
%逆%速%度%计%算%
%%%%%%%%%%%%%%%
%求雅可比矩阵
z00=[0,0,1]' ; z01=R{1}*z00 ; z02=R{1}*R{2}*z00;
p0=[0,0,0]' ; 
p1=R{1}*P{1} ;
p2=R{1}*P{2}+p1;
p3=R{1}*R{2}*P{3}+p2;

j_b1=cross(z00,(p3-p0));
j_b2=cross(z01,(p3-p1));
j_b3=cross(z02,(p3-p2));
jacobian=[j_b1 , j_b2 , j_b3];


%%%%%%%%%%%%%%%%
%逆速度计算
%%%%%%%%%%%%%%%%
end_vol=[Pdx,Pdy,Pdz]';%末端速度（m/s；rad/s），在世界坐标系中表示
j11=jacobian(:,1) ; j12=jacobian(:,2) ; j13=jacobian(:,3);
j21=jacobian(:,1)'*jacobian(:,1); j22=jacobian(:,1)'* jacobian(:,2);
j23=jacobian(:,1)'* jacobian(:,3);
j31=jacobian(:,2)'*jacobian(:,1); j32=jacobian(:,2)'*jacobian(:,2);
j33=jacobian(:,2)'*jacobian(:,3); 
n1=j11 ; n2=-j12*j21+j11*j22;
n3= - j13*j22*j31 + j12*j23*j31 + j13*j21*j32 - j11*j23*j32 - j12*j21*j33 + j11*j22*j33;
qd3=(end_vol'*n3) / (jacobian(:,3)'*n3);
qd2=((end_vol-jacobian(:,3)*qd3)'*n2 )/ (jacobian(:,2)'*n2);
qd1=( (end_vol - (jacobian(:,3)*qd3 + jacobian(:,2)*qd2  ))'*n1) / (jacobian(:,1)'*n1) ;




%%%%%%%%%%%%%
%正速度计算
%%%%%%%%%%%%%
p1sta=[0,0,0]';p2sta=[l2,0,0]';p3sta=[l3,0,0]';
omega00=[0,0,0]';
omega11=R{1}'*(omega00+[0,0,1]'*qd1);
omega22=R{2}'*(omega11+[0,0,1]'*qd2);
omega33=R{3}'*(omega22+[0,0,1]'*qd3);
v00=[0,0,0]';
v11=R{1}'*v00+cross(omega11,p1sta);
v22=R{2}'*v11+cross(omega22,p2sta);
v33=R{3}'*v22+cross(omega33,p3sta);
v03=R{1}*R{2}*R{3}*v33;

%%%%%%%%%%%%%%%%
%雅可比矩阵的导数
%%%%%%%%%%%%%%%%
omega01=R{1}*omega11;
omega02=R{1}*R{2}*omega22;
v01=R{1}*v11 ; v02=R{1}*R{2}*v22;
db1=cross((cross(omega00,z00)),(p3-p0))+cross(z00,(v03-v00));
db2=cross((cross(omega01,z01)),(p3-p1))+cross(z01,(v03-v01));
db3=cross((cross(omega02,z02)),(p3-p2))+cross(z02,(v03-v02));
dj=[db1,db2,db3];


%%%%%%%%%%%%%
%加速度逆解算
%%%%%%%%%%%%%
xdds=[Pddx,Pddy,Pddz]'-dj*[qd1,qd2,qd3]';
qdd3=(xdds'*n3) / (jacobian(:,3)'*n3);
qdd2=((xdds-jacobian(:,3)*qdd3)'*n2 )/ (jacobian(:,2)'*n2);
qdd1=( (xdds - (jacobian(:,3)*qdd3 + jacobian(:,2)*qdd2  ))'*n1) / (jacobian(:,1)'*n1) ;







sys(1)=q1;
sys(2)=q2;
sys(3)=q3;
sys(4)=qd1;
sys(5)=qd2;
sys(6)=qd3;
sys(7)=qdd1;
sys(8)=qdd2;
sys(9)=qdd3;