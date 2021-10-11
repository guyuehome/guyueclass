%IENDPOINT Find end points in a binary skeleton image
%
% OUT = IENDPOINT(IM) is a binary image where pixels are set if the
% corresponding pixel in the binary image IM is the end point of a 
% single-pixel wide line such as found in an image skeleton.  Computed 
% using the hit-or-miss morphological operator.
%
% References::
%  - Robotics, Vision & Control, Section 12.5.3
%    P. Corke, Springer 2011.
%
% See also ITRIPLEPOINT, ITHIN, HITORMISS.


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

function o = iendpoint(im)

    o = im;
    se(:,:,1) = [   0 1 0
                    0 1 0
                    0 0 0   ];
                
    se(:,:,2) = [   0 0 1
                    0 1 0
                    0 0 0   ];
                
    se(:,:,3) = [   0 0 0
                    0 1 1
                    0 0 0   ];
                
    se(:,:,4) = [   0 0 0
                    0 1 0
                    0 0 1   ];
                
    se(:,:,5) = [   0 0 0
                    0 1 0
                    0 1 0   ];
                
    se(:,:,6) = [   0 0 0
                    0 1 0
                    1 0 0   ];
                
    se(:,:,7) = [   0 0 0
                    1 1 0
                    0 0 0   ];
                
    se(:,:,8) = [   1 0 0
                    0 1 0
                    0 0 0   ];

    o = zeros(size(im));
    for i=1:8
        o = o | hitormiss(im, se(:,:,i));
    end
