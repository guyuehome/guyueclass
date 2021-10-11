%ILINE Draw a line in an image
%
% OUT = ILINE(IM, P1, P2) is a copy of the image IM with a single-pixel thick
% line drawn between the points P1 and P2, each a 2-vector [U,V].  The pixels 
% on the line are set to 1.
%
% OUT = ILINE(IM, P1, P2, V) as above but the pixels on the line are set to V.
%
% Notes::
% - Uses the Bresenham algorithm.
% - Only works for greyscale images.
% - The line looks jagged since no anti-aliasing is performed.
%
% See also BRESENHAM, IPROFILE, IPASTE.


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

function c2 = iline(c, p1, p2, value)

    if nargin < 4
        value = 1;
    end

    points = bresenham(p1, p2);

    c2 = c;
    for point = points'
        c2(point(2), point(1)) = value;
    end
