function [axis,angle]=rotation2axisangle(R)
cosq = 0.5*(R(1,1)+R(2,2)+R(3,3)-1);
if(cosq>1)
    cosq=1;
end
if(cosq<-1)
    cosq=-1;
end
angle = acos(cosq);
axis = zeros(3,1);
if(abs(angle)<1e-4)    
elseif(abs(angle-pi)<1e-4)
    for i=1:3
        a = 0.5*(R(i,i)+1);
        if abs(a)<1e-4
            axis(i) = 0;
        else
            axis(i) = a^0.5;
        end
    end
    
    maxind = 1;
    if axis(2)>axis(1)
        maxind = 2;
    end
    if axis(3)>axis(2)
        maxind = 3;
    end
    if(maxind == 1)
        a1a2 = R(1,2) + R(2,1);
        a1a3 = R(1,3) + R(3,1);
        if(a1a2<0)
            axis(2) = -axis(2);
        end
        if(a1a3<0)
            axis(3) = -axis(3);
        end
    end
    if(maxind == 2)
        a1a2 = R(1,2) + R(2,1);
        a2a3 = R(2,3) + R(3,2);
        if(a1a2<0)
            axis(1) = -axis(1);
        end
        if(a2a3<0)
            axis(3) = -axis(3);
        end
    end
    if(maxind == 3)
        a1a3 = R(1,3) + R(3,1);
        a2a3 = R(2,3) + R(3,2);
        if(a1a3<0)
            axis(1) = -axis(1);
        end
        if(a2a3<0)
            axis(2) = -axis(2);
        end
    end
    axis = axis/norm(axis);
else
    axis(1) = R(3,2) - R(2,3);
    axis(2) = R(1,3) - R(3,1);
    axis(3) = R(2,1) - R(1,2);
    axis = axis/norm(axis);
end

end