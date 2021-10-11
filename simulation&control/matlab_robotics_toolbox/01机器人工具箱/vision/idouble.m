%IDOUBLE Convert integer image to double
%
% IMD = IDOUBLE(IM) is an image with double precision elements in the
% range 0 to 1 corresponding to the elements of IM. The integer pixels IM
% are assumed to span the range 0 to the maximum value of their integer class.
%
% Notes::
% - Works for an image with arbitrary number of dimensions, eg. a color
%   image or image sequence.
% - There is a linear mapping (scaling) of the values of IMD to IM.
%
% See also IINT, CAST.


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

function dim = idouble(im, opt)

    if nargin == 2 && strcmp(opt, 'float')
        % convert to float pixel values
        if isinteger(im)
            dim = single(im) / single(intmax(class(im)));
        elseif islogical(im)
            dim = single(im);
        else
            dim = im;
        end
    else
        % convert to double pixel values (default)
        if isinteger(im)
            dim = double(im) / double(intmax(class(im)));
        elseif islogical(im)
            dim = double(im);
        else
            dim = im;
        end
    end
    

