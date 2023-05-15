function [Xout] = runge_kutta4(ufunc,Xin,u,t,dt,i)% ode45
h       = dt;
[k1]    = ufunc(Xin,u,t,i);
[k2]    = ufunc(Xin+h*k1/2,u,t+h/2,i);
[k3]    = ufunc(Xin+h*k2/2,u,t+h/2,i);
[k4]    = ufunc(Xin+h*k3,u,t+h,i);

[Xout]  = Xin+h*(k1+2*k2+2*k3+k4)/6;
end