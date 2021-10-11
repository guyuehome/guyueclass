%INVCAMCAL	Inverse camera calibration
%
%	c = INVCAMCAL(C)
%
% 	Decompose, or invert, a 3x4camera calibration matrix C.
%   The result is a camera object with the following parameters set:
%      f
%      sx, sy  (with sx=1)
%      (u0, v0)  principal point
%	   Tcam is the homog xform of the world origin wrt camera
%
% Since only f.sx and f.sy can be estimated we set sx = 1.
%
% REF:	Multiple View Geometry, Hartley&Zisserman, p 163-164
%
% SEE ALSO: camera



% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

function c = invcamcal(C)

    if ~all(size(C) == [3 4]),
        error('argument is not a 3x4 matrix');
    end
    [u,s,v] = svd(C);

    t = v(:,4);
    t = t / t(4);
    t = t(1:3);

    M = C(1:3,1:3);
    [K,R] = vgg_rq(M);

    % normalize K so that lower left is 1
    K = K/K(3,3);

    f = K(1,1);
    s = [1 K(2,2)/K(1,1)];
    if f < 0,
        f = -f;
        s = -s;
    end
    c = CentralCamera( 'name', 'invcamcal', ...
        'focal', f, ...
        'centre', K(1:2,3), ...
        'pixel', s, ...
        'pose', [R t; 0 0 0 1] );
