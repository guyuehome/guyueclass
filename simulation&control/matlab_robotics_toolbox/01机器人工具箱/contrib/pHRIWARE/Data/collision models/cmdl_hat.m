%CMDL_HAT Create a CollisionModel object of the human HAT
%
% Returns a CollisionModel object of the human head and torso segment
% (also includes neck). The model may be given for any shoulder frame
% transformation (synonymous with the base transform of a HAL object).
% The head is a sphere, the neck a cylinder, the shoulders an ellipsoid
% and the torso an elliptical cylinder.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) hat = cmdl_hat(Tg)
%  (2) hat = cmdl_hat()
%
%  (2) is as per (1), using HAL's default value of Tg
%
% Outputs:
%  hat : CollisionModel object of the human HAT
%
% Inputs:
%  Tg : Transformation frame of the shoulder
%
% See also anthroData cmdl_arm HAL

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
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

function hat = cmdl_hat(Tg)

if ~nargin, Tg = trotx(pi/2); end

skin = pHRIWARE('skin');
shirt = pHRIWARE('orange');

% Anthropometric Data
[refnote, pb, sb, pn, sn, pg, sg, pt, st] = ...
    anthroData('pb', 'rb', 'pn', 'sn', 'pg', 'sg', 'pt', 'st');

% Head
gTb = [eye(3), pb; 0, 0, 0, 1];
Tb = Tg * gTb;
Head = Sphere(Tb, sb, 'FaceColor', skin, 'EdgeColor', 'none');

% Neck
gTn = [rotx(-pi/2), pn; 0, 0, 0, 1];
Tn = Tg * gTn;
Neck = Cylinder(Tn, sn, 'FaceColor', skin, 'EdgeColor', 'none');

% Shoulders
gT = [eye(3), pg; 0, 0, 0, 1];
T = Tg * gT;
Shoulders = Ellipsoid(T, sg, 'FaceColor', shirt, 'EdgeColor', 'none');

% Torso
gTt = [rotx(pi/2), pt; 0, 0, 0, 1];
Tt = Tg * gTt;
Torso = Cylinder(Tt, st, 'FaceColor', shirt, 'EdgeColor', 'none');

hat = CollisionModel(refnote, Head, Neck, Shoulders, Torso);

end

