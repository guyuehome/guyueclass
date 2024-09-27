function [R,T,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,A)

% EFFICIENT_PNP Main Function to solve the PnP problem 
%       as described in:
%
%       Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua.
%       Accurate Non-Iterative O(n) Solution to the PnP Problem. 
%       In Proceedings of ICCV, 2007. 
%
%       x3d_h: homogeneous coordinates of the points in world reference
%       x2d_h: homogeneous position of the points in the image plane
%       A: intrincic camera parameters
%       R: Rotation of the camera system wrt world reference
%       T: Translation of the camera system wrt world reference
%       Xc: Position of the points in the camera reference
%
% Copyright (C) <2007>  <Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua>
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the version 3 of the GNU General Public License
% as published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.       
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%
% Francesc Moreno-Noguer, CVLab-EPFL, September 2007.
% fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 


Xw=x3d_h(:,1:3);
U=x2d_h(:,1:2);

THRESHOLD_REPROJECTION_ERROR=20;%error in degrees of the basis formed by the control points. 
%If we have a larger error, we will compute the solution using a larger
%number of vectors in the kernel

%define control points in a world coordinate system (centered on the 3d
%points centroid)
Cw=define_control_points();

%compute alphas (linear combination of the control points to represent the 3d
%points)
Alph=compute_alphas(Xw,Cw);

%Compute M
M=compute_M_ver2(U,Alph,A);

%Compute kernel M
Km=kernel_noise(M,4); %in matlab we have directly the funcion km=null(M);
    


%1.-Solve assuming dim(ker(M))=1. X=[Km_end];------------------------------
dim_kerM=1;
X1=Km(:,end);
[Cc,Xc]=compute_norm_sign_scaling_factor(X1,Cw,Alph,Xw);

[R,T]=getrotT(Xw,Xc);  %solve exterior orientation
err(1)=reprojection_error_usingRT(Xw,U,R,T,A);

sol(1).Xc=Xc;
sol(1).Cc=Cc;
sol(1).R=R;
sol(1).T=T;
sol(1).error=err(1);


%2.-Solve assuming dim(ker(M))=2------------------------------------------
Km1=Km(:,end-1);
Km2=Km(:,end);

%control points distance constraint
D=compute_constraint_distance_2param_6eq_3unk(Km1,Km2);
dsq=define_distances_btw_control_points();
betas_=inv(D'*D)*D'*dsq;
beta1=sqrt(abs(betas_(1)));
beta2=sqrt(abs(betas_(3)))*sign(betas_(2))*sign(betas_(1));
X2=beta1*Km1+beta2*Km2;

[Cc,Xc]=compute_norm_sign_scaling_factor(X2,Cw,Alph,Xw);

[R,T]=getrotT(Xw,Xc);  %solve exterior orientation
err(2)=reprojection_error_usingRT(Xw,U,R,T,A);

sol(2).Xc=Xc;
sol(2).Cc=Cc;
sol(2).R=R;
sol(2).T=T;
sol(2).error=err(2);



%3.-Solve assuming dim(ker(M))=3------------------------------------------
if min(err)>THRESHOLD_REPROJECTION_ERROR %just compute if we do not have good solution in the previus cases

    Km1=Km(:,end-2);
    Km2=Km(:,end-1);
    Km3=Km(:,end);

    %control points distance constraint
    D=compute_constraint_distance_3param_6eq_6unk(Km1,Km2,Km3);
    dsq=define_distances_btw_control_points();
    betas_=inv(D)*dsq;
    beta1=sqrt(abs(betas_(1)));
    beta2=sqrt(abs(betas_(4)))*sign(betas_(2))*sign(betas_(1));
    beta3=sqrt(abs(betas_(6)))*sign(betas_(3))*sign(betas_(1));

    X3=beta1*Km1+beta2*Km2+beta3*Km3;

    [Cc,Xc]=compute_norm_sign_scaling_factor(X3,Cw,Alph,Xw);
  
    [R,T]=getrotT(Xw,Xc);  %solve exterior orientation
    err(3)=reprojection_error_usingRT(Xw,U,R,T,A);

    sol(3).Xc=Xc;
    sol(3).Cc=Cc;
    sol(3).R=R;
    sol(3).T=T;
    sol(3).error=err(3);

end



%4.-Solve assuming dim(ker(M))=4------------------------------------------
if min(err)>THRESHOLD_REPROJECTION_ERROR %just compute if we do not have good solution in the previus cases
    Km1=Km(:,end-3);
    Km2=Km(:,end-2);
    Km3=Km(:,end-1);
    Km4=Km(:,end);


    D=compute_constraint_distance_orthog_4param_9eq_10unk(Km1,Km2,Km3,Km4);
    dsq=define_distances_btw_control_points();
    lastcolumn=[-dsq',0,0,0]';
    D_=[D,lastcolumn];
    Kd=null(D_);

    P=compute_permutation_constraint4(Kd);
    lambdas_=kernel_noise(P,1);
    lambda(1)=sqrt(abs(lambdas_(1)));
    lambda(2)=sqrt(abs(lambdas_(6)))*sign(lambdas_(2))*sign(lambdas_(1));
    lambda(3)=sqrt(abs(lambdas_(10)))*sign(lambdas_(3))*sign(lambdas_(1));
    lambda(4)=sqrt(abs(lambdas_(13)))*sign(lambdas_(4))*sign(lambdas_(1));
    lambda(5)=sqrt(abs(lambdas_(15)))*sign(lambdas_(5))*sign(lambdas_(1));

    betass_=lambda(1)*Kd(:,1)+lambda(2)*Kd(:,2)+lambda(3)*Kd(:,3)+lambda(4)*Kd(:,4)+lambda(5)*Kd(:,5);
    beta1=sqrt(abs(betass_(1)));
    beta2=sqrt(abs(betass_(5)))*sign(betass_(2));
    beta3=sqrt(abs(betass_(8)))*sign(betass_(3));
    beta4=sqrt(abs(betass_(10)))*sign(betass_(4));
    X4=beta1*Km1+beta2*Km2+beta3*Km3+beta4*Km4;

    [Cc,Xc]=compute_norm_sign_scaling_factor(X4,Cw,Alph,Xw);
    
    [R,T]=getrotT(Xw,Xc);  %solve exterior orientation
    err(4)=reprojection_error_usingRT(Xw,U,R,T,A);

    sol(4).Xc=Xc;
    sol(4).Cc=Cc;
    sol(4).R=R;
    sol(4).T=T;
    sol(4).error=err(4);

end

[min_err,best_solution]=min(err);
Xc=sol(best_solution).Xc;
R=sol(best_solution).R;
T=sol(best_solution).T;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R, T]=getrotT(wpts,cpts)
  
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
% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [err,Urep]=reprojection_error_usingRT(Xw,U,R,T,A)

%clear all; close all; load reprojection_error_usingRT;
n=size(Xw,1);

P=A*[R,T];
Xw_h=[Xw,ones(n,1)];
Urep_=(P*Xw_h')';

%project reference points into the image plane
Urep=zeros(n,2);
Urep(:,1)=Urep_(:,1)./Urep_(:,3);
Urep(:,2)=Urep_(:,2)./Urep_(:,3);

%reprojection error
err_=sqrt((U(:,1)-Urep(:,1)).^2+(U(:,2)-Urep(:,2)).^2);
err=sum(err_)/n;