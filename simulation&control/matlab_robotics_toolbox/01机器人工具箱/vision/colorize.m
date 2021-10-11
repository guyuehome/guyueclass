%COLORIZE Colorize a greyscale image
%
% OUT = COLORIZE(IM, MASK, COLOR) is a color image where each pixel in OUT
% is set to the corresponding element of the greyscale image IM or a specified
% COLOR according to whether the corresponding value of MASK is true or false 
% respectively.  The color is specified as a 3-vector (R,G,B).
%
% OUT = COLORIZE(IM, FUNC, COLOR) as above but a the mask is the return value
% of the function handle FUNC applied to the image IM, and returns a per-pixel
% logical result, eg. @isnan.
%
% Examples::
%
% Display image with values < 100 in blue
%    out = colorize(im, im<100, [0 0 1])
%
% Display image with NaN values shown in red
%    out = colorize(im, @isnan, [1 0 0])
%
% Notes::
% - With no output arguments the image is displayed.
%
% See also IMONO, ICOLOR, IPIXSWITCH.



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

function co = colorize(img, mask, color)

    if isa(img, 'uint8') || isa(img, 'logical') 
        grey = double(img) / 255;
    else
        grey = img / max(img(:));
    end

    g = grey(:);
    z = [g g g];
    if isa(mask, 'function_handle')
        mask = mask(img);
    end

    k = find(mask(:));
    z(k,:) = ones(numrows(k),1)*color(:)';
    z = reshape(z, [size(grey) 3]);

    if nargout > 0
        co = z;
    else
        image(z);
        shg
    end
