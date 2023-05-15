function y = runge_kutta4(ufunc,Xin,u,h,i)
% runge_kutta4
    k1 = ufunc(Xin,u,i);
    k2 = ufunc(Xin+h/2*k1,u,i);
    k3 = ufunc(Xin+h/2*k2,u,i);
    k4 = ufunc(Xin+h*k3,u,i);
    y  = Xin+h/6*(k1+2*k2+2*k3+k4); 
end