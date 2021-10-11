%KCIRCLE Circular structuring element
%
% K = KCIRCLE(R) is a square matrix (WxW) where W=2R+1 of zeros with a maximal 
% centred circular region of radius R pixels set to one.
%
% K = KCIRCLE(R,W) as above but the dimension of the kernel is explicitly 
% specified.
%
% Notes::
% - If R is a 2-element vector the result is an annulus of ones, and
%   the two numbers are interpretted as inner and outer radii.
%
% See also ONES, KTRIANGLE, IMORPH.



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

function s = kcircle(r, w)

    if ~isscalar(r) 
        rmax = max(r(:));
        rmin = min(r(:));
    else
        rmax = r;
    end


    if nargin == 2
        w = w*2 + 1;
    elseif nargin == 1
        w = 2*rmax+1;
    end
    s = zeros(w,w);

    c = ceil(w/2);

    if ~isscalar(r) 
        s = kcircle(rmax,w) - kcircle(rmin, w);
    else
        [x,y] = imeshgrid(s);
        x = x - c;
        y = y - c;
        l = find(round(x.^2 + y.^2 - r^2) <= 0);
        s(l) = 1;
    end
