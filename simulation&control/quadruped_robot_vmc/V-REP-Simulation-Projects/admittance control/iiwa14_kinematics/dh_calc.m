function A = dh_calc(a,alpha,d,theta)
% DH_CALC
%
% Calculates the homogeneous transformation between two consecutive joint frames
%
% Input:  [a, alpha, d, theta] - classical DH parameters (not modified DH)
% Output: A - homogeneous transformation between two consecutive joint frames

v = [ a*cos(theta), a*sin(theta), d ];
Xx = cos(theta); Yx = -sin(theta) * cos(alpha); Zx =  sin(theta) * sin(alpha);
Xy = sin(theta); Yy =  cos(theta) * cos(alpha); Zy = -cos(theta) * sin(alpha);
Xz = 0.0;        Yz =  sin(alpha);              Zz =  cos(alpha); 
A = [Xx, Yx, Zx, v(1);
     Xy, Yy, Zy, v(2);
     Xz, Yz, Zz, v(3);
     0,  0,  0,  1];
end