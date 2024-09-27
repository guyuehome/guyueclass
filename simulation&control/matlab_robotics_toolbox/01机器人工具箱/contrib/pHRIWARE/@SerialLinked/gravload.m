% GRAVLOAD Joint loads due to gravity
% 
% Uses either the rne MEX file, from RTB, or the grav function, from
% pHRIWARE, depending on being able to use the MEX file and if the
% Jacobian is requested to be returned
% 
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
% 
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
% 
% Syntax:
%  (1) tauB = robot.gravload(q)
%  (2) [tauB, J] = robot.grav(q)
%  (3) ... = robot.grav(q, grav)
% 
%  (2) is as per (1) but also returns world-frame Jacobian - pHRIWARE
%  (3) is as per others but with explicit gravity vector
% 
% Outputs:
%  tauB : Generalised joint force/torques
%  J    : Jacobian in world frame
% 
% Inputs:
%  q     : Joint row vector, or trajectory matrix of joint row vectors
%  grav  : Gravity _reaction_ vector (as RTB default is [0 0 +9.81]')
%
% See also grav, rne

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
%
% RTB LIBRARY:
%
% Copyright (C) 1993-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license

function varargout = gravload(robot, varargin)

if robot.fast && ~robot.issym() && nargout <= 1 
   qz = zeros(size(varargin{1}));
   rne_input = {varargin(1), qz, qz, varargin(2:end)};
   varargout = robot.frne(rne_input{:});
else 
   pHRIWARE('c');
   varargout = robot.grav(varargin{:}); 
end 

end