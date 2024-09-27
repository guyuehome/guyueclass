%CMDL_ARM Create a CollisionModel object of the human arm
%
% Returns a CollisionModel object of the human upper arm and forearm.
% Link lengths may be specified or anthropometric data can be used. The
% Upper arm is a cylinder, and the forearm is a cylindrical frustum
% (Curvilinear), to accomodate for the change in thickness of the 
% forearm from the elbow to the wrist.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) arm = cmdl_arm(gre, erw)
%  (2) arm = cmdl_arm()
%
%  (2) is as per (1), using data from anthroData
%
% Outputs:
%  arm : CollisionModel object of the human arm
%
% Inputs:
%  gre : Upper arm length
%  erw : Forearm length
%
% See also anthroData cmdl_hat

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

function arm = cmdl_arm(gre, erw)

skin = pHRIWARE('skin');
shirt = pHRIWARE('orange');

% Anthropometric Data
[refnote, rw, re, ru, greAD, erwAD] = ...
    anthroData('rw', 're', 'ru', 'gre', 'erw');

if nargin == 0
    gre = greAD;
    erw = erwAD;
end

% Upper arm
Tu = trotx(-pi/2); % Transform to cylinder upperarm frame
su = [ru, ru, gre];
Upperarm = Cylinder(Tu, su, 'FaceColor', shirt, 'EdgeColor', 'none');

% Forearm
Tf = troty(-pi/2); % Transform to curvilinear forearm frame
sf = [1, 1, erw];
rf = @(t) rw + t*(re-rw);
Forearm = Curvilinear(Tf,sf,rf, 'FaceColor',skin, 'EdgeColor','none');

arm = CollisionModel(refnote, Upperarm, Forearm);

end

