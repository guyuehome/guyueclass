%COL2IM Convert pixel vector to image
%
% OUT = COL2IM(PIX, IMSIZE) is an image (HxWxP) comprising the pixel values in 
% PIX (NxP) with one row per pixel where N=HxW.  IMSIZE is a 2-vector (N,M).
%
% OUT = COL2IM(PIX, IM) as above but the dimensions of OUT are the same as IM.
%
% Notes::
% - The number of rows in PIX must match the product of the elements of IMSIZE.
%
% See also IM2COL.



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

function im = col2im(col, img)

    ncols = numcols(col);

    if numel(img) == 2
        sz = img;
    elseif numel(img) == 3
        sz = img(1:2);
    else
        sz = size(img);
        sz = sz(1:2);
    end

    if ncols > 1
        sz = [sz ncols];
    end

    im = reshape(col, sz);
