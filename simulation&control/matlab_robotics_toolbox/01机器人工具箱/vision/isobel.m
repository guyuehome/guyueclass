%ISOBEL Sobel edge detector
%
% OUT = ISOBEL(IM) is an edge image computed using the Sobel edge operator
% applied to the image IM.  This is the norm of the vertical and horizontal 
% gradients at each pixel.  The Sobel horizontal gradient kernel is:
%        | -1  0  1|
%        | -2  0  2|
%        | -1  0  1|
%
% and the vertical gradient kernel is the transpose.
%
% [GX,GY] = ISOBEL(IM) as above but returns the gradient images rather than
% the gradient magnitude.
%
% OUT = ISOBEL(IM,DX) as above but applies the kernel DX and DX' to compute
% the horizontal and vertical gradients respectively.
%
% [GX,GY] = ISOBEL(IM,DX) as above but returns the gradient images rather than
% the gradient magnitude.
%
% Notes::
% - Tends to produce quite thick edges.
% - The resulting image is the same size as the input image.
% - If the kernel DX is provided it can be of any size, not just 3x3, and
%   could be generated using KDGAUSS.
%
% See also KSOBEL, KDGAUSS, ICANNY, ICONV.


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

function [o1,o2] = isobel(im, varargin)

    % convert color image to greyscale
    if iscolor(im)
        im = imono(im);
    end
    
    opt.size = {'same', 'full', 'valid'};
    opt.smooth = [];
    [opt,args] = tb_optparse(opt, varargin);
    
    
    if size(im, 3) > 1
        error('MVTB:isobel:badarg', 'not defined for multiplane image');
    end
    if length(args) == 0
        Dx = [ -1 -2 -1
            0 0 0
            1 2 1];
    else
        % use the passed horizontal gradient kernel
        Dx = args{1};
    end
    if ~isempty(opt.smooth)
        Dx = kdgauss(opt.smooth);
    end

    ih = iconv(im, Dx, opt.size);
    iv = iconv(im, Dx', opt.size);

    % return grandient components or magnitude
    if nargout == 1,
        o1 = sqrt(ih.^2 + iv.^2);
    else
        o1 = ih;
        o2 = iv;
    end
