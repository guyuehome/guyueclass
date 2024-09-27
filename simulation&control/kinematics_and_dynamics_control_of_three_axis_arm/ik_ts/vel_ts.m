
clc; clear;
l1 = 0; 
l2 = 0.13;
l3 = 0.13;
elbow=-1;
Px=0.15;     Py=0;    Pz=0.0;
Pdx=0.013;    Pdy=-0.007;   Pdz=-0.01;
Pddx=0.005;   Pddy=0;  Pddz=0;
    

if Pz==0
    %已知三角形三边就三个角度
    r2 = Px * Px + Py * Py + Pz * Pz;
    r = sqrt(r2);
    q2 = acos((l2*l2+r2-l3*l3) / (2*r*l2));
    q3 = acos((l2*l2+l3*l3-r2) / (2*l3*l2))-pi;%互补
    q1 = atan2(Py, Px);
else


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
end

%旋转矩阵
R={[cos(q1) 0 sin(q1)  ; sin(q1)  0 -cos(q1); 0 1 0],...
   [cos(q2) -sin(q2) 0 ; sin(q2) cos(q2) 0  ; 0 0 1],... 
   [cos(q3) -sin(q3) 0 ; sin(q3) cos(q3) 0  ; 0 0 1],... 
   };
%原点坐标
P={[0 0 0]' , [l2*cos(q2) l2*sin(q2) 0]' , [l3*cos(q3) l3*sin(q3) 0]' };
%P{0}坐标系1的位置在坐标系0的表示
%P{1}坐标系2的位置在坐标系1的表示
%P{2}坐标系3的位置在坐标系2的表示

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

%{
n1 = j11

n2 = 
j11 j12
j21 j22

n3 = 
j11 j12 j13
j21 j22 j23
j31 j32 j33
%}
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

v03=R{1}*R{2}*R{3}*v33



%点乘
p1 = [1 2 3];
p2 = [3 2 1];
% p1*p2'
%叉乘
% s_p1=[0 -p1(3) p1(2);p1(3) 0 -p1(1);-p1(2) p1(1) 0];
% s_p1*p2';
% cross(p1,p2);
% 
% s_p2=[0 -p2(3) p2(2);p2(3) 0 -p2(1);-p2(2) p2(1) 0];
% s_p2*p1'
% cross(p2,p1)








