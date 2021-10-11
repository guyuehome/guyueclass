%IINT Convert image to integer class
%
% OUT = IINT(IM) is an image with unsigned 8-bit integer elements in the
% range 0 to 255 corresponding to the elements of the image IM. 
%
% OUT = IINT(IM, CLASS) as above but the output pixels belong to the
% integer class CLASS.
%
% Examples::
%   Convert double precision image to 8-bit unsigned integer
%         im = rand(50, 50);
%         out = iint(im);
%
%   Convert double precision image to 16-bit unsigned integer
%         im = rand(50, 50);
%         out = iint(im, 'uint16');
%
%   Convert 8-bit unsigned integer image to 16-bit unsigned integer
%         im = randi(255, 50, 50, 'uint8');
%         out = iint(im, 'uint16');
%
% Notes::
% - Works for an image with arbitrary number of dimensions, eg. a color
%   image or image sequence.
% - If the input image is floating point (single or double) the pixel values 
%   are scaled from an input range of [0,1] to a range spanning zero to the
%   maximum positive value of the output integer class.
% - If the input image is an integer class then the pixels are cast to 
%   change type but not their value.
%
% See also IDOUBLE.


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

% OUT = IINT(IM, CLASS) returns an image with integer elements of the specified
% class in the range 0 INTMAX.  CLASS is a string representing any of the 
% standard Matlab integer classes, eg. 'int16'.  The floating point pixels are 
% assumed to span the range 0 to 1.
%
% Examples::
%
%    im = iint(dim, 'int16');
%
% See also IDOUBLE, CLASS, INTMAX.

function im = iint(in, cls)

    if nargin < 2
        cls = 'uint8';
    end

    if isfloat(in)
        % rescale to integer
        im = cast(round( in * double(intmax(cls))), cls);
    else
        im = cast(in, cls);
    end
