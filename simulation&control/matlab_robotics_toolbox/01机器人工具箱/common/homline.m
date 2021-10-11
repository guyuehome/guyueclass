%HOMLINE Homogeneous line from two points
%
% L = HOMLINE(X1, Y1, X2, Y2) is a vector (3x1) which describes a line in
% homogeneous form that contains the two Euclidean points (X1,Y1) and (X2,Y2).
%
% Homogeneous points X (3x1) on the line must satisfy L'*X = 0.
%
% See also PLOT_HOMLINE.

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
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
%
% http://www.petercorke.com

% TODO, probably should be part of a HomLine class.

function l = homline(x1, y1, x2, y2)

    l = cross([x1 y1 1], [x2 y2 1]);

    % normalize so that the result of x*l' is the pixel distance
    % from the line
    l = l / norm(l(1:2));
