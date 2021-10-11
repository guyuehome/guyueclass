%CentralCamera.estpose Estimate pose from object model and camera view
%
% T = C.estpose(XYZ, UV) is an estimate of the pose of the object defined by
% coordinates XYZ (3xN) in its own coordinate frame.  UV (2xN) are the 
% corresponding image plane coordinates.
%
% Reference::
% "EPnP: An accurate O(n) solution to the PnP problem",
% V. Lepetit, F. Moreno-Noguer, and P. Fua,
% Int. Journal on Computer Vision,
% vol. 81, pp. 155-166, Feb. 2009.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function T = estpose(c, XYZ, uv)

    [R, t] = efficient_pnp(XYZ', uv', c.K);

    T = [R t; 0 0 0 1];
