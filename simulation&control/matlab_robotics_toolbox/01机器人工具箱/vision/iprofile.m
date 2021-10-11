%IPROFILE Extract pixels along a line
%
% V = IPROFILE(IM, P1, P2) is a vector of pixel values extracted from the
% image IM (HxWxP) between the points P1 (2x1) and P2 (2x1).  V (NxP) has 
% one row for each point along the line and the row is the pixel value 
% which will be a vector for a multi-plane image.
%
% [P,UV] = IPROFILE(IM, P1, P2) as above but also returns the coordinates of
% the pixels for each point along the line.  Each row of UV is the pixel 
% coordinate (u,v) for the corresponding row of P.
%
% Notes::
% - The Bresenham algorithm is used to find points along the line.
%
% See also BRESENHAM, ILINE.


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

function [p,uv] = iprofile(c, p1, p2)

    % coordinates must be integers
    p1 = round(p1); p2 = round(p2);
    
    points = bresenham(p1, p2);

    p = [];
    for point = points'
        p = [p; c(point(2), point(1), :)];
    end

    if nargout > 1
        uv = squeeze(points');
    end
