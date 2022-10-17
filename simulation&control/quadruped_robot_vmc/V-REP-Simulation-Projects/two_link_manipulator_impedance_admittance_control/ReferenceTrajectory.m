function [pref,vref,aref] = ReferenceTrajectory(t,trajectory_mode)
% horizontal line trajectory -- constant velocity
% pstart = [-0.4098;0.1098];
% pend = [0.4098;0.1098];
% T = 30;
% aref = [0;0];
% vref = (pend - pstart)/T;
% pref = pstart + vref*t;
% if t>T
%     vref = [0;0];
%     pref = pend;
% end
if trajectory_mode == 0
    % horizontal line trajectory -- five order polynomial interpolation
    pstart = [-0.4098;0.1098];
    pend = [0.4098;0.1098];
    T = 30;
    if t>T
        pref = [0.4098;0.1098];
        vref = [0;0];
        aref = [0;0];
    else
        l = norm(pend - pstart);
        k = (pend - pstart)/l;
        a3 = 10*l/T^3;a4 = -15*l/T^4;a5 = 6*l/T^5;
        pref = pstart + k*(a3*t^3 + a4*t^4 + a5*t^5);
        vref = k*(3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4);
        aref = k*(6*a3*t + 12*a4*t^2 + 20*a5*t^3);
    end
elseif trajectory_mode  ==1
    % circle trajectory -- constant magnitude of cartesian velocity
    T = 60;
    angle = pi;
    omega = [0;angle/T;0];
    % pstart = [-0.4098;0;0.1098];
    alpha = 1;
    pstart = [-0.4098;0;0.1098];
    pref = pstart*(1-alpha) + roty(6*t)*pstart*alpha;
    vref = cross(omega,pref);
    aref = cross(vref,omega);
    pref = [pref(1);pref(3)];
    vref = [vref(1);vref(3)];
    aref = [aref(1);aref(3)];
    if t>T
        vref = [0;0];
        aref = [0;0];
        pref = -[-0.4098;0.1098];
    end
end
%  circle trajectory -- various cartesian velocity

end