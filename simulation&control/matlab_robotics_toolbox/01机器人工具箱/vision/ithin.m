%ITHIN Morphological skeletonization
%
% OUT = ITHIN(IM) is the binary skeleton of the binary image IM.  Any non-zero 
% region is replaced by a network of single-pixel wide lines.
%
% OUT = ITHIN(IM,DELAY) as above but graphically displays each iteration 
% of the skeletonization algorithm with a pause of DELAY seconds between 
% each iteration.
%
% References::
%  - Robotics, Vision & Control, Section 12.5.3,
%    P. Corke, Springer 2011.
%
% See also HITORMISS, ITRIPLEPOINT, IENDPOINT.


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

function out = ithin(im, delay)

    % create a binary image
    im = im > 0;
    
    o = im;

    Sa = [0 0 0; NaN 1 NaN; 1 1 1];
    Sb = [NaN 0 0; 1 1 0; NaN 1 NaN];

    o = im;
    while true
        for i=1:4
            r = hitormiss(im, Sa);
            im = im - r;
            r = hitormiss(im, Sb);
            im = im - r;
            Sa = rot90(Sa);
            Sb = rot90(Sb);
        end
        if nargin > 1
            idisp(im);
            pause(delay);
        end
        if all(o == im)
            break;
        end
        o = im;
    end
    o = im;
    if nargout > 0
        out = o;
    end
