points=[];
scale=0.2;
pZ=0.8;
offset =[1.2,0,0]
for t=0:0.1:2*3.14
	s=sin(t)
	c=cos(t)
	f=s*abs(c)^0.5/(s+1.4)-2*s+2
	x=f*c;
	y=f*s;
	z = pZ;
    Tp=[scale*y scale*x z]+offset;
    points=[points;Tp];
end

for t=0:0.1:2*3.14 
	a=1.3;
	k=2
	x = a*(k*cos(t)-cos(k*t));
    y = a*(k*sin(t)-sin(k*t));
	z = pZ;
    Tp=[scale*x scale*y z]+offset;
     points=[points;Tp];
end
px=points(:,1);
py=points(:,2);
pz=points(:,3);
plot3(px,py,pz)