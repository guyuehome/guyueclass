function dsq=define_distances_btw_control_points()

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


%relative coordinates of the control points
c1=[1,0,0];
c2=[0,1,0];
c3=[0,0,1];
c4=[0,0,0];

d12=(c1(1)-c2(1))^2 + (c1(2)-c2(2))^2 + (c1(3)-c2(3))^2;
d13=(c1(1)-c3(1))^2 + (c1(2)-c3(2))^2 + (c1(3)-c3(3))^2;
d14=(c1(1)-c4(1))^2 + (c1(2)-c4(2))^2 + (c1(3)-c4(3))^2;
d23=(c2(1)-c3(1))^2 + (c2(2)-c3(2))^2 + (c2(3)-c3(3))^2;
d24=(c2(1)-c4(1))^2 + (c2(2)-c4(2))^2 + (c2(3)-c4(3))^2;
d34=(c3(1)-c4(1))^2 + (c3(2)-c4(2))^2 + (c3(3)-c4(3))^2;

dsq=[d12,d13,d14,d23,d24,d34]';