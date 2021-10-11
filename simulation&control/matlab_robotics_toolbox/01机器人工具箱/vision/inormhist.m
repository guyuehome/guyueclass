%INORMHIST Histogram normalization
%
% OUT = INORMHIST(IM) is a histogram normalized version of the image IM.
%
% Notes::
% - Highlights image detail in dark areas of an image.
% - The histogram of the normalized image is approximately uniform, that is,
%   all grey levels ae equally likely to occur.
%
% See also IHIST.



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
function [ni,ch] = inormhist(im)
    if size(im,3) > 1
        error('inormhist doesnt support color images');
    end
    [cdf,x] = ihist(im, 'cdf');
    [nr,nc] = size(im);
    cdf = cdf/max(cdf);
    
    if isfloat(im)
        ni = interp1(x', cdf', im(:), 'nearest', 'extrap');
    else
        ni = interp1(x', cdf', double(im(:)), 'nearest');
        ni = cast(ni*double(intmax(class(im))), class(im));
    end
    ni = reshape(ni, nr, nc);
