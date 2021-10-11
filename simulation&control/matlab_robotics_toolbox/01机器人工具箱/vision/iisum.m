%IISUM Sum of integral image
%
% S = IISUM(II, U1, V1, U2, V2) is the sum of pixels in the rectangular image
% region defined by its top-left (U1,V1) and bottom-right (U2,V2).  II is
% a precomputed integral image.
%
% See also INTGIMAGE.


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

function s = iisum(ii, c1, r1, c2, r2)

    r1 = r1 - 1;
    if r1 < 1
        sA = 0;
        sB = 0;
    else
        sB = ii(r1,c2);
    end
    c1 = c1 - 1;
    if c1 < 1
        sA = 0;
        sC = 0;
    else
        sC = ii(r2,c1);
    end
    if (r1 >= 1) && (c1 >= 1)
        sA = ii(r1,c1);
    end

    s = ii(r2,c2) + sA -sB - sC;
