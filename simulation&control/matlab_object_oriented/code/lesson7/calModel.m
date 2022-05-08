function z=calModel(x)
n=size(x,1);
z=zeros(n,2);
for i=1:2
    z(:,1)=z(:,1)-10*exp(-0.2*sqrt(x(:,i).^2+x(:,i+1).^2));
end
for i=1:3
    z(:,2)=z(:,2)+abs(x(:,i)).^0.8+5*sin(x(:,i).^3);
end
end