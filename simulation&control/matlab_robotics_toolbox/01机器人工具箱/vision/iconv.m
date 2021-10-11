%ICONV Image cross-correlation
%
% C = ICONV(IM1, IM2, OPTIONS) is the cross-correlation of images IM1 and IM2.  The 
% smaller image is taken as the kernel and correlated with the larger image. 
%
% Options::
%  'same'    output image is same size as largest input image (default)
%  'full'    output image is larger than the input image
%  'valid'   output image is smaller than the input image, and contains only
%            valid pixels
%
% Notes::
% - If the larger image is color (has multiple planes) the kernel is applied to 
%   each plane, resulting in an output image with the same number of planes.
% - The kernel must be greyscale.
% - This function is a convenience wrapper for the MATLAB function CONV2.
% - Works for double, uint8 or uint16 images.  Image and kernel must be of
%   the same type and the result is of the same type.
% - This function is badly named, it peforms cross-correlation not
%   convolution.
%
% See also CONV2.


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
function C = iconv(A, B, opt)

    if nargin < 3
        opt = 'same';
    end

    if numcols(A) < numcols(B)
        % B is the image
        A = fliplr(flipud(A));
        for k=1:size(B,3)
            C(:,:,k) = conv2(B(:,:,k), A, opt);
        end
    else
        % A is the image
        B = fliplr(flipud(B));
        for k=1:size(A,3)
            C(:,:,k) = conv2(A(:,:,k), B, opt);
        end
    end

