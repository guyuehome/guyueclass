function [issingular,sols] = SolveSphericalWrist(joints,R_des,robot_type)
qlower = -[170,120,170,120,170,120,175]*pi/180;
qupper = -qlower;
issingular = false;
sols=[];
[J,A_mat_products] = Jacobian( joints, robot_type );
A04 = A_mat_products{4};
R04 = A04(1:3,1:3);
R47 = R04'*R_des;
q5=0;q6=0;q7=0;
% select branch
wrist_sign = 1;
if joints(6)<0
    wrist_sign = -1;
end
if(abs(abs(R47(3,3))-1)<1e-3)
    q5 = joints(5);
    issingular = true;
else
    q5 = atan2(wrist_sign*R47(2,3),wrist_sign*R47(1,3));
end
q6 = wrist_sign*acos(R47(3,3));
q7 = atan2(wrist_sign*R47(3,2),-wrist_sign*R47(3,1));
if fit_q_to_range(qlower(5),qupper(5),q5)&&fit_q_to_range(qlower(6),qupper(6),q6)&&fit_q_to_range(qlower(7),qupper(7),q7)
    sol = joints;
    sol(5) = q5;
    sol(6) = q6;
    sol(7) = q7;
    sols=[sols,sol];
end

if q5<0
    q5 = q5 + pi;
else
    q5 = q5 - pi;
end
q6 = -q6;
if q7 <0
    q7 = q7 + pi;
else
    q7 = q7 - pi;
end
if fit_q_to_range(qlower(5),qupper(5),q5)&&fit_q_to_range(qlower(6),qupper(6),q6)&&fit_q_to_range(qlower(7),qupper(7),q7)
    sol = joints;
    sol(5) = q5;
    sol(6) = q6;
    sol(7) = q7;
    sols=[sols,sol];
end
end