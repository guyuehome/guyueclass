function [Beta_opt,err,iter]=gauss_newton(K,Cw,Beta0)

% GAUSS_NEWTON  
%
%       K 12x4:  Matrix of kernel vectors
%       Cw 4x3:  Coordinates of the control points in world reference
%       Beta0: initial estimation of Beta's
%
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
% Francesc Moreno-Noguer, CVLab-EPFL, October 2007.
% fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 

L=compute_L6_10(K);
rho=compute_rho(Cw);

current_betas=Beta0;
  
n_iterations=10; %max number of iterations. Usually 4-5 is enought.
for k=1:n_iterations
    [A,b]=compute_A_and_b_Gauss_Newton(current_betas,rho,L);
    dbeta=inv(A'*A)*A'*b;
    current_betas=current_betas+dbeta';
    error(k)=b'*b;
    iter(k).betas=current_betas;
    iter(k).error=error(k);    
end

Beta_opt=current_betas;
err=error(end);


