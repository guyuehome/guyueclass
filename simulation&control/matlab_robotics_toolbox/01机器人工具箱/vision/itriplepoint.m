%ITRIPLEPOINT Find triple points
%
% OUT = ITRIPLEPOINT(IM) is a binary image where pixels are set if the
% corresponding pixel in the binary image IM is a triple point, that is where 
% three single-pixel wide line intersect.  These are the Voronoi points in 
% an image skeleton.  Computed using the hit-or-miss morphological operator.
%
% References::
%  - Robotics, Vision & Control, Section 12.5.3,
%    P. Corke, Springer 2011.
%
% See also IENDPOINT, ITHIN, HITORMISS.


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

function o = triplepoint(im)

    o = im;

    se(:,:,1) = [   0  1 0   ;...
                     1  1  1   ;...
                    0 0 0   ];

    se(:,:,2) = [    1 0  1   ;...
                    0  1 0   ;...
                    0 0  1   ];

    se(:,:,3) = [   0  1 0   ;...
                    0  1  1   ;...
                    0  1 0   ];

    se(:,:,4) = [   0 0  1   ;...
                    0  1 0   ;...
                     1 0  1   ];

    se(:,:,5) = [   0 0 0   ;...
                     1  1  1   ;...
                    0  1 0   ];

    se(:,:,6) = [    1 0 0   ;...
                    0  1 0   ;...
                     1 0  1   ];

    se(:,:,7) = [   0 1 0
                    1 1 0
                    0 1 0   ];

    se(:,:,8) = [   1 0 1
                    0 1 0
                    1 0 0   ];

    se(:,:,9) = [   0 1 0
                    0 1 1
                    1 0 0   ];

    se(:,:,10)= [   0 0 1
                    1  1 0
                    0 0 1   ];

    se(:,:,11)= [   1 0 0
                    0 1 1
                    0 1 0   ];

    se(:,:,12)= [   0 1 0
                    0 1 0
                    1 0 1   ];

    se(:,:,13)= [   0 0 1
                    1 1 0
                    0 1 0   ];

    se(:,:,14)= [   1 0 0
                    0 1 1
                    1 0 0   ];

    se(:,:,15)= [   0 1 0
                    1 1 0
                    0 0 1   ];

    se(:,:,16)= [   1 0 1
                    0 1 0
                    0 1 0   ];

    o = zeros(size(im));
    for i=1:16
        o = o | hitormiss(im, se(:,:,i));
    end
