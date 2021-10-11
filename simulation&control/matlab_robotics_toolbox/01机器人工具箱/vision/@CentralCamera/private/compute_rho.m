function rho=compute_rho(Cw)

% COMPUTE_RHO

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


c0=Cw(1,:);
c1=Cw(2,:);
c2=Cw(3,:);
c3=Cw(4,:);

rho(1) = dist2(c0,c1);
rho(2) = dist2(c0,c2);
rho(3) = dist2(c0,c3);
rho(4) = dist2(c1,c2);
rho(5) = dist2(c1,c3);
rho(6) = dist2(c2,c3);

