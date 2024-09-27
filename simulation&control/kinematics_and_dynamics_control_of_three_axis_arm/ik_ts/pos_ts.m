
clc; clear;

l1 = 0; 
l2 = 0.13;
l3 = 0.13;
elbow=-1;


Px=0.13;
Py=-0.03;
Pz=-0.01;
fprintf('期望末端坐标：\n');
show_pos = [Px Py Pz]

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
    q2 = q21 + q22;

    if Pz<0
        q2 = q21 - q22;
    end
    %%%%%%%%%%%%%%
end




%旋转矩阵
R={[cos(q1) 0 sin(q1)  ; sin(q1)  0 -cos(q1); 0 1 0],...
   [cos(q2) -sin(q2) 0 ; sin(q2) cos(q2) 0  ; 0 0 1],... 
   [cos(q3) -sin(q3) 0 ; sin(q3) cos(q3) 0  ; 0 0 1],... 
   };

%原点坐标
P={[0 0 0]' , [l2*cos(q2) l2*sin(q2) 0]' , [l3*cos(q3) l3*sin(q3) 0]' };



fprintf('验证的末端坐标：\n');

p1=R{1}*P{1} ;
p2=R{1}*P{2} + p1;
p3=R{1}*R{2}*P{3} + p2;
p3'



