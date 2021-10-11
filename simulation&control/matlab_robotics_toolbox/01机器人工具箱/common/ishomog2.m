%ISHOMOG2 Test if SE(2) homogeneous transformation
%
% ISHOMOG2(T) is true (1) if the argument T is of dimension 3x3 or 3x3xN, else 
% false (0).
%
% ISHOMOG2(T, 'valid') as above, but also checks the validity of the rotation
% sub-matrix.
%
% Notes::
% - The first form is a fast, but incomplete, test for a transform in SE(3).
% - Does not work for the SE(3) case.
%
% See also ISHOMOG, ISROT2, ISVEC.



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

function h = ishomog2(tr, rtest)
    d = size(tr);
    if ndims(tr) >= 2
        h =  all(d(1:2) == [3 3]);

        if h && nargin > 1
            h = abs(det(tr(1:2,1:2)) - 1) < eps;
        end
    else
        h = false;
    end
