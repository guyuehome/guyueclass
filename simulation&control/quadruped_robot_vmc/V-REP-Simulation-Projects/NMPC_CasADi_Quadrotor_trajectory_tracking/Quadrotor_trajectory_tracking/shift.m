function [t0, x0, u0] = shift(T, t0, x0, u,f)
st = x0;
con = u(1,:)';
% euler
% f_value = f(st,con);
% st = st + f_value*T;
% RK4
f_value1 = f(st,con);
f_value2 = f(st+T/2*f_value1,con);
f_value3 = f(st+T/2*f_value2,con);
f_value4 = f(st+T*f_value3,con);
st = st+ (f_value1+2*f_value2+2*f_value3+f_value4)*T/6;
% update x0 t0 u0
x0 = full(st);
t0 = t0 + T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end