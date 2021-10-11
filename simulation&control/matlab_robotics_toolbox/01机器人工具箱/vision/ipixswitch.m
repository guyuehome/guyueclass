%IPIXSWITCH Pixelwise image merge
%
% OUT = IPIXSWITCH(MASK, IM1, IM2) is an image where each pixel is
% selected from the corresponding pixel in IM1 or IM2 according to the
% corresponding pixel values in MASK.  If the element of MASK is zero IM1 is
% selected, otherwise IM2 is selected.
%
% IM1 or IM2 can contain a color descriptor which is one of:
% - A scalar value corresponding to a greyscale
% - A 3-vector corresponding to a color value
% - A string containing the name of a color which is found using COLORNAME.
%
% IPIXSWITCH(MASK, IM1, IM2) as above but the result is displayed.
%
% Example::
%  Read a uint8 image
%         im = iread('lena.pgm');
%  and set high valued pixels to red
%         a = ipixswitch(im>120, im, uint8([255 0 0]));
%  The result is a uint8 image since both arguments are uint8 images.
%
%         a = ipixswitch(im>120, im, [1 0 0]);
%  The result is a double precision image since the color specification
%  is a double.
%
%         a = ipixswitch(im>120, im, 'red');
%  The result is a double precision image since the result of colorname
%  is a double precision 3-vector.
%
% Notes::
% - IM1, IM2 and MASK must all have the same number of rows and columns.
% - If IM1 and IM2 are both greyscale then OUT is greyscale.
% - If either of IM1 and IM2 are color then OUT is color.
% - If either one image is double and one is integer then the integer
%   image is first converted to a double image.
%
% See also COLORIZE, COLORNAME.


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

function co = ipixswitch(mask, I1, I2)

    if ischar(I1)
        % image is a string color name
        col = colorname(I1);
        if isempty(col)
            error('unknown color %s', col);
        end
        I1 = icolor(ones(size(mask)), col);
    elseif isscalar(I1)
        % image is a scalar, create a greyscale image same size as mask
        I1 = ones(size(mask), class(I1))*I1;
    elseif ndims(I1) == 2 && all(size(I1) == [1 3])
        % image is 1x3, create a color image same size as mask
        I1 = icolor(ones(size(mask), class(I1)), I1);
    else
        % actual image, check the dims
        s = size(I1); s = s(1:2);
        if ~all(s == size(mask))
            error('input image sizes do not conform');
        end
    end

    if ischar(I2)
        % image is a string color name
        col = colorname(I2);
        if isempty(col)
            error('unknown color %s', col);
        end
        I2 = icolor(ones(size(mask)), col);
    elseif isscalar(I2)
        % image is a scalar, create a greyscale image same size as mask
        I2 = ones(size(mask), class(I2))*I2;
    elseif ndims(I2) == 2 && all(size(I2) == [1 3])
        % image is 1x3, create a color image same size as mask
        I2 = icolor(ones(size(mask), class(I2)), I2);
    else
        % actual image, check the dims
        s = size(I2); s = s(1:2);
        if ~all(s == size(mask))
            error('input image sizes do not conform');
        end
    end

    if isfloat(I1) && isinteger(I2)
        I2 = idouble(I2);
    elseif isinteger(I1) && isfloat(I2)
        I1 = idouble(I1);
    end

    nplanes = max(size(I1,3), size(I2,3));

    if nplanes == 3
        mask = repmat(mask, [1 1 3]);
        if size(I1,3) == 1
            I1 = repmat(I1, [1 1 3]);
        end
        if size(I2,3) == 1
            I2 = repmat(I2, [1 1 3]);
        end
    end

    % in case one of the images contains NaNs we cant blend the images
    % using arithmetic
    % out = mask .* I1 + (1-mask) .* I2;
    out = I2;
    out(mask) = I1(mask);

    if nargout > 0
        co = out;
    else
        idisp(out);
        shg
    end
