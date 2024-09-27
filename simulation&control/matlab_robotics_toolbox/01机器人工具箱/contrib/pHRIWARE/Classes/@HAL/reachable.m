%REACHABLE Find if a pose is reachable
%
% Tests if a pose, described by two sets of joint angles (such as that
% from ikine), is reachable or not. Will output the permutation of the
% two sets, which is reachable.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) q = hal.islimit(q1, q2)
%  (2) qu = hal.islimit(qu1, qu2)
%  (3) [..., u] = ...
%
%  (2) is as per (1) but operates on the shoulder only
%  (3) is as per (1) or (2) but with vector marking unreachable poses
%
% Outputs:
%  q :  HAL joint angles. Each row is a pose and may contain parts of
%       q1 and q2, depending if they are within limits. If the pose is
%       not reachable, that row is NaN
%  qu : See q, but only shoulder angles (q(1:3))
%  u  : Vector which is equal to any(isnan(q),2). I.e, elements are
%       true if that row of q is unreachable
%
% Inputs:
%  q1  : HAL joint angles - first family of solutions
%  q2  : HAL joint angles - Second family of solutions
%  qu1 : See q1, but only shoulder angles (q1(1:3))
%  qu2 : See q2, but only shoulder angles (q2(1:3))
%
% See also HAL.ikine HAL.islimit

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

function [q, u] = reachable(hal, q1, q2)

q = NaN(size(q1));

if size(q1,2) == 3 % For upper arm case, set others to zero
    q1(:,4:7) = 0;
    q2(:,4:7) = 0;
end
    
I1 = hal.islimit(q1); % Find the angles which are out of limits
I2 = hal.islimit(q2); % For the two families

q1g = all(~I1(:,1:3),2); % Get rows which all shoulder joints are false
q2g = all(~I2(:,1:3),2); % i.e. within limit, so valid shoulder config
q(q2g,1:3) = q2(q2g,1:3); % Place the valid shoulder joint sets in q
q(q1g,1:3) = q1(q1g,1:3); % Family 1 overwrites family 2 if both good

% Do the same for elbow and wrist
if size(q,2) == 7
    q1e = all(~I1(:,4),2);
    q1w = all(~I1(:,5:7),2);
    q2e = all(~I2(:,4),2);
    q2w = all(~I2(:,5:7),2);
    q(q2e,4) = q2(q2e,4);
    q(q2w,5:7) = q2(q2w,5:7);
    q(q1e,4) = q1(q1e,4);
    q(q1w,5:7) = q1(q1w,5:7);
end

u = any(isnan(q),2);

q(u,:) = NaN;

end

