function s=sign_determinant(C)

% SIGN_DETERMINANT  

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

c0=C(4,:)';
c1=C(1,:)';
c2=C(2,:)';
c3=C(3,:)';


v1=c1-c0;
v2=c2-c0;
v3=c3-c0;

M=[v1,v2,v3];
detM=det(M);
s=sign(detM);
