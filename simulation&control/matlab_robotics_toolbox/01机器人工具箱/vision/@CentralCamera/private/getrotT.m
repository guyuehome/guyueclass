function [R, T]=getrotT(wpts,cpts);
  
% This routine solves the exterior orientation problem for a point cloud
%  given in both camera and world coordinates. 
  
% wpts = 3D points in arbitrary reference frame
% cpts = 3D points in camera reference frame
  
n=size(wpts,1);
M=zeros(3);

ccent=mean(cpts);
wcent=mean(wpts);

for i=1:3
  cpts(:,i)=cpts(:,i)-ccent(i)*ones(n,1);
  wpts(:,i)=wpts(:,i)-wcent(i)*ones(n,1);
end

for i=1:n
   M=M+cpts(i,:)'*wpts(i,:);
end

[U S V]=svd(M);
R=U*V';
if det(R)<0
  R=-R;
end

T=ccent'-R*wcent';

