function sol = Inverse_Kinematics(q,pd,l)
% 
px = pd(1);
py = pd(2);
l1 = l(1);
l2 = l(2);
distance = px^2 + py^2;
if distance > (l1+l2)^2 
    k = pd/norm(pd);
    p = (l1+l2)*k;
    px = p(1);
    py = p(2);
elseif distance < (l1-l2)^2
    k = pd/norm(pd);
    p = (l1-l2)*k;
    px = p(1);
    py = p(2);
end

cosq2 = (distance - l1^2 - l2^2)/2/l1/l2;
sinq2 = sqrt(1-cosq2^2);
sincosq2 = zeros(2,2);
sincosq2(:,1) = [sinq2;cosq2];
sincosq2(:,2) = [-sinq2;cosq2];
solutions = zeros(2,4);
for i=1:2
    sinq2 = sincosq2(1,i);
    cosq2 = sincosq2(2,i);
    A = l1 + l2*cosq2;
    B = l2*sinq2;
    sinq1 = (px*A - py*B)/(A^2+B^2);
    cosq1 = (px*B + py*A)/(A^2+B^2);
    q1 = atan2(sinq1,cosq1);
    q2 = atan2(sinq2,cosq2);
    solutions(:,i) = [q1;q2];
end

if solutions(1,1) > solutions(1,2)
    solutions(1,3) = solutions(1,1) - 2*pi;
    solutions(1,4) = solutions(1,2) + 2*pi;
else
    solutions(1,3) = solutions(1,1) + 2*pi;
    solutions(1,4) = solutions(1,2) - 2*pi;
end

if solutions(2,1) > solutions(2,2)
    solutions(2,3) = solutions(2,1) - 2*pi;
    solutions(2,4) = solutions(2,2) + 2*pi;
else
    solutions(2,3) = solutions(2,1) + 2*pi;
    solutions(2,4) = solutions(2,2) - 2*pi;
end

delta = 10000;
index = 0;
for i=1:4
    if norm(q - solutions(:,i))<delta
        delta = norm(q - solutions(:,i));
        index = i;
    end
end
sol = solutions(:,index);
end