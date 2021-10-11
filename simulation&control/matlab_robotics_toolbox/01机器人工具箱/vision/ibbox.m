%IBBOX Find bounding box
%
% BOX = IBBOX(P) is the minimal bounding box that contains the points
% described by the columns of P (2xN).
%
% BOX = IBBOX(IM) as above but the box minimally contains the non-zero
% pixels in the image IM.
%
% Notes::
% - The bounding box is a 2x2 matrix [XMIN XMAX; YMIN YMAX].


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

function box = ibbox(I)

    if numrows(I) == 2
        % input is a set of points
        u = I(1,:);
        v = I(2,:);
    else
        % input is an image, find the non-zero elements
        [v,u] = find(I);
    end
    umin = min(u);
    umax = max(u);
    vmin = min(v);
    vmax = max(v);

    box = [umin umax; vmin vmax];
