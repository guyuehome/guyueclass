%GIKINE Shoulder inverse kinematics of HAL-like right shoulder
%
% Computes the inverse kinematics of the right shoulder which is the 
% same kinematically as a HAL object. This function is mainly useful 
% for procedures which require many, many calls, so time can be saved 
% by not referencing a HAL object.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) [q1, q2] = gikine(Tg, Tu)
%
% Outputs:
%  q1 : First family of solutions (mx3 matrix where m = size(Tu,3))
%  q2 : Second family of solutions (mx3 matrix where m = size(Tu,3))
%
% Inputs:
%  Tg : Transformation matrix of the shoulder frame. x, y, z point to
%        the right, above, and behind the person. Translation is
%        shoulder center of rotation
%  Tu : Transformation matrix of the upper arm frame. May be a 4x4xm
%        series of frames (or higher order, which is compressed to 3D)
%
% See also HAL HAL.gikine HAL.ikine wikine

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
%% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.
%
% RTB LIBRARY:
%
% Copyright (C) 199q3-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license,
% Modified 16/7/2014 (HAL is a subclass of SerialLink)

function [q1, q2] = gikine(Tg, Tu)

Rg = Tg(1:3,1:3);
gRu0 = [0 0 -1; 0 1 0; 1 0 0]';
u0R = (Rg*gRu0)';

u0Ru = reshape(u0R*reshape(Tu(1:3,1:3,:),3,[]),3,3,[]);

%-X,Z,Y Tait-Bryan angles

% Family 1:
q1(:,1) = squeeze(atan2(-u0Ru(3,2,:),u0Ru(2,2,:)));
q1(:,2) = squeeze(asin(-u0Ru(1,2,:)));
q1(:,3) = squeeze(atan2(-u0Ru(1,3,:),u0Ru(1,1,:)));

% Make -pi values pi so that they are deemed within joint limits
fix = q1 <= (-pi+1e-4);
q1(fix) = pi;

% Family 2:
q2(:,1) = squeeze(atan2(u0Ru(3,2,:),-u0Ru(2,2,:)));
q2(:,2) = pi-q1(:,2);
q2(:,3) = squeeze(atan2(u0Ru(1,3,:),-u0Ru(1,1,:)));
end