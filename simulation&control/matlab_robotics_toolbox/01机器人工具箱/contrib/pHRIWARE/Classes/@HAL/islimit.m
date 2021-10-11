%ISLIMIT Test if HAL joint angles are within their limits
%
% Returns a matrix the same size as the matrix of joint angles input
% (where each row is a joint angle set), whose elements are false if
% within joint limits or true if not. The formulae by Lenarcic & Umek
% (1994) are used to calculate the shoulder range of motion, with the
% qlim values representing the limits when other joint angles are zero.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Upper arm range of motion:
%  Lenarcic, J. & Umek, A. 1994, 'Simple Model of Human Arm Reachable 
%   Workspace', IEEE Transactions on systems, man, and cybernetics, 
%   vol. 24, no. 8, pp. 1239-46. 
%
% Syntax:
%  (1) I = hal.islimit(q)
%
% Outputs:
%  I : logical matrix, elements which are true are angles outside limit
%
% Inputs:
%  q : Joint row vector, or trajectory matrix of joint row vectors
%
% See also HAL.reachable

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

function I = islimit(hal, q)

I = false(size(q));

for i = [1 4 5 6 7]
    I(:,i) = hal.qlim(i,2) <= q(:,i)|q(:,i) <= hal.qlim(i,1);
end

toplim(:,1) = hal.qlim(2,2) - q(:,1)/6;
toplim(:,2) = hal.qlim(3,2) + ...
    4*q(:,1)/9 - 5*q(:,2)/9 + 10*q(:,1).*q(:,2)/(9*pi);

lowlim(:,1) = hal.qlim(2,1) + q(:,1)/3;
lowlim(:,2) = hal.qlim(3,1) + ...
    7*q(:,1)/9 - q(:,2)/9 + 4*q(:,1).*q(:,2)/(9*pi);

I(:,[2 3]) =  toplim <= q(:,[2 3])|q(:,[2 3]) <= lowlim;

end

