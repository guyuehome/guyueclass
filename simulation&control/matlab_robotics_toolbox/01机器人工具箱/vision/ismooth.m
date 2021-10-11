%ISMOOTH Gaussian smoothing
%
% OUT = ISMOOTH(IM, SIGMA) is the image IM after convolution with a
% Gaussian kernel of standard deviation SIGMA.
%
% OUT = ISMOOTH(IM, SIGMA, OPTIONS) as above but the OPTIONS are passed
% to CONV2.
%
% Options::
% 'full'    returns the full 2-D convolution (default)
% 'same'    returns OUT the same size as IM
% 'valid'   returns  the valid pixels only, those where the kernel does not
%           exceed the bounds of the image.
%
% Notes::
% - By default (option 'full') the returned image is larger than the
%   passed image.
% - Smooths all planes of the input image.
% - The Gaussian kernel has a unit volume.
% - If input image is integer it is converted to float, convolved, then
%   converted back to integer.
%
% See also ICONV, KGAUSS.



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

function ims = ismooth(im, sigma, varargin)

    if isfloat(im)
        is_int = false;
    else
        is_int = true;
        im = idouble(im);
    end

    m = kgauss(sigma, varargin{:});

    for i=1:size(im,3),
        ims(:,:,i) = conv2(im(:,:,i), m, 'same');
    end

    if is_int
        ims = iint(ims);
     end

    if nargout == 0
        idisp(ims);
    end
