%RADGRAD Radial gradient
%
% [GR,GT] = RADGRAD(IM) is the radial and tangential gradient of the image IM.
% At each pixel the image gradient vector is resolved into the radial and 
% tangential directions.
%
% [GR,GT] = RADGRAD(IM, CENTRE) as above but the centre of the image is
% specified as CENTRE=[X,Y] rather than the centre pixel of IM.
%
% RADGRAD(IM) as above but the result is displayed graphically.
%
% See also ISOBEL.



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

function [gr,gt] = radgrad(im, center)
    
    [nr,nc] =size(im);

    if nargin == 1
        xc = nr/2;
        yc = nc/2;
    else
        xc = center(1);
        yc = center(2);
    end

    [X,Y] = meshgrid(1:nc, 1:nr);
    X = X - xc;
    Y = Y - yc;
    H = sqrt(X.^2 + Y.^2);
    sth = Y ./ H;
    cth = X ./ H;
    [ih,iv] = isobel(im);

    g = sth .* iv + cth .* ih;

    if nargout == 0
        idisp(g);
    elseif nargout == 1
        gr = g;
    elseif nargout == 2
        gr = g;
        gt = cth .* iv + sth .* ih;
    end
