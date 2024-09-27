%IKINE Inverse kinematics of HAL objects
%
% Executes the inverse kinematics of a HAL (human arm-like) object.
% It returns the two sets of possible solutions for the shoulder and 
% wrist in two of the four possible permutations - the other two may be
% created from the others.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) [q1, q2] = hal.ikine(Th, phi)
%  (2) [q1, q2] = hal.ikine(Th)
%
%  (2) is as per (1) but uses phi := (1)
%
% Outputs:
%  q1 : First family of solutions (mx7 matrix where m = size(Th,3))
%  q2 : Second family of solutions (mx7 matrix where m = size(Th,3))
%
% Inputs:
%  Th  : Hand frame(s), may be a 4x4xm array of m frames
%  phi : Swivel angle,
%        (0) 0     : Uses phi = 0
%        (1)       : Uses the z-axis of Tw for the z-axis of Ts, via
%                     projection
%        (2) 'h2g' : Is the hand-2-goal method, s.t. z-axis of Ts is 
%                     the cross product of the swivel axis and g
%        (3) step  : step is a scalar >0 so that the swivel angle
%                      is added as an an extra dimension, with phi = 
%                      median human range +/- step increments up to the
%                      human limits. The swivel angle increases with 
%                      the increasing index of the extra dimension.
%        (4) PHI   : PHI uses an explicit value, the number of elements
%                     in PHI must be a multiple of the number of hand 
%                     points/frames
%
% See also h2fsu HAL.gikine HAL.wikine

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
% Released under the GNU Lesser General Public license,
% Modified 16/6/2014 (HAL is a subclass of SerialLink)

function [q1, q2] = ikine(hal, Th, phi)

if nargin == 2
    [Tf, ~, Tu] = hal.h2fsu(Th);
else
    [Tf, ~, Tu] = hal.h2fsu(Th, phi);
end

[q1g, q2g] = hal.gikine(Tu);

qe = squeeze(pi/2 - acos(dot(Tu(1:3,2,:),Tf(1:3,1,:))));

[q1w, q2w] = wikine(Th, Tf);

q1 = [q1g, qe, q1w];
q2 = [q2g, qe, q2w];

end

