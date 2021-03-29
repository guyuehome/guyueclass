function [gamma,alfa,beta] = xyz(x,y,z)
h=0.15;
hu=0.35;
hl=0.382;
dyz=sqrt(y.^2+z.^2);
lyz=sqrt(dyz.^2-h.^2);
gamma_yz=-atan(y/z);
gamma_h_offset=-atan(h./lyz);
gamma=gamma_yz-gamma_h_offset;
%
lxzp=sqrt(lyz.^2+x.^2);
n=(lxzp.^2-hl.^2-hu.^2)/(2*hu);
beta=-acos(n/hl);

%
alfa_xzp=-atan(x/lyz);
alfa_off=acos((hu+n)/lxzp);
alfa=alfa_xzp+alfa_off;

%输出角度为弧度
end

