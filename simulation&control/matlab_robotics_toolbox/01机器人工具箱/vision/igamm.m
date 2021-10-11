%IGAMM	Gamma correction
%
% OUT = IGAMM(IM, GAMMA) is a gamma corrected version of the image IM.  All 
% pixels are raised to the power GAMMA.  Gamma encoding can be performed with 
% GAMMA > 1 and decoding with GAMMA < 1.
%
% OUT = IGAMM(IM, 'sRGB') is a gamma decoded version of IM using the sRGB 
% decoding function (JPEG images sRGB encoded).
%
% Notes::
% - This function was once called igamma(), but that name taken by MATLAB
%   method for double class objects.
% - Gamma decoding should be applied to any color image prior to colometric
%   operations.
% - The exception to this is colorspace conversion using COLORSPACE which
%   expects RGB images to be gamma encoded.
% - Gamma encoding is typically performed in a camera with GAMMA=0.45.
% - Gamma decoding is typically performed in the display with GAMMA=2.2.
% - For images with multiple planes the gamma correction is applied to all 
%   planes.
% - For images sequences the gamma correction is applied to all elements.
% - For images of type double the pixels are assumed to be in the range 0 to 1.
% - For images of type int the pixels are assumed in the range 0 to the maximum
%   value of their class.  Pixels are converted first to double, processed,
%   then converted back to the integer class.
%
% See also IREAD, COLORSPACE.



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

function g = igamm(im, gam)

    if strcmpi(gam, 'srgb')
        % sRGB decompress
        if isfloat(im)
            f = im;
        else
            % int image
            maxval = intmax(class(im));
            f = (double(im) / double(maxval));
        end

        % convert gamma encoded sRGB to linear tristimulus values
        k = f <= 0.04045;
        f(k) = f(k) / 12.92;
        k = f > 0.04045;
        f(k) = ( (f(k)+0.055)/1.055 ).^2.4;
        g = f;
        if ~isfloat(im)
            g = cast(g*double(maxval), class(im));
        end
    else
        % normal power law
        if isfloat(im)
            % float image
            g = im .^ gam;
        else
            % int image
            maxval = double(intmax(class(im)));
            g = ((double(im) / maxval) .^ gam) * maxval;
            g = cast(g, class(im));
        end
    end
