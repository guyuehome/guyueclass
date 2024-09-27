L1=Link([0 0.445 0.150 -pi/2 0]);
L2=Link ([-pi/2 0 0.700 0 0]);
L3=Link([0 0 0.115 -pi/2 0]);
L4=Link([0 0.795 0 pi/2 0]);
L5=Link([0 0 0 -pi/2 0]);
L6=Link([0 0.085 0 0 0]);
L=[L1,L2,L3,L4,L5,L6];
IRB2600 = SerialLink(L,'name','ABB IRB 2600');

X=1;
Y=1;
Z=1;
%¸ø¶¨Î»×Ë
P=[X,Y,Z];
Ts=transl(P);
IRB2600.tool=trotx(pi);
TA=IRB2600.A([1 2 3 4 5 6],[0 0 0 0 0 0]) ;

q= IRB2600.ikunc(TA*Ts);
IRB2600.plot(q)


