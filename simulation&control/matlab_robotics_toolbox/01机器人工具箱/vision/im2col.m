%IM2COL Convert an image to pixel per row format
%
% OUT = IM2COL(IM) is a matrix (NxP) where each row represents a single
% of the image IM (HxWxP).  The pixels are in image column order (ie. column
% 1, column 2 etc) and there are N=WxH rows.
%
% OUT = IM2COL(IM, MASK) as above but only includes pixels if:
% - the corresponding element of MASK (HxW) is non-zero
% - the corresponding element of MASK (N) is non-zero where N=HxW
% - the pixel index is included in the vector MASK
%
% See also COL2IM.



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

function c = im2col(im, mask)

    if ndims(im) == 3
        c = reshape(shiftdim(im, 2), 3, [])';
    else
        c = reshape(im, 1, [])';
    end
    
    if nargin > 1 && ~isempty(mask)
        d = size(im);
        if ndims(mask) == 2 && all(d(1:2) == size(mask))
            k = find(mask);
        elseif isvector(mask)
            k = mask;
        else
            error('MVTB:im2col:badarg', 'mask must be same size as image or a vector');
        end
        
        c = c(k,:);
    end
