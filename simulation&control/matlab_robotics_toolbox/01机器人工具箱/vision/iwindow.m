%IWINDOW Generalized spatial operator
%
% OUT = IWINDOW(IM, SE, FUNC) is an image where each pixel is the result
% of applying the function FUNC to a neighbourhood centred on the corresponding
% pixel in IM.  The neighbourhood is defined by the size of the structuring
% element SE which should have odd side lengths.  The elements in the 
% neighbourhood corresponding to non-zero elements in SE are packed into
% a vector (in column order from top left) and passed to the specified
% function handle FUNC.  The return value  becomes the corresponding pixel 
% value in OUT.
%
% OUT = IWINDOW(IMAGE, SE, FUNC, EDGE) as above but performance of edge 
% pixels can be controlled.  The value of EDGE is:
% 'border'   the border value is replicated (default)
% 'none'     pixels beyond the border are not included in the window
% 'trim'     output is not computed for pixels whose window crosses
%            the border, hence output image had reduced dimensions.
% 'wrap'     the image is assumed to wrap around
%
% Example::
% Compute the maximum value over a 5x5 window:
%      iwindow(im, ones(5,5), @max);
%
% Compute the standard deviation over a 3x3 window:
%      iwindow(im, ones(3,3), @std);
%
% Notes::
% - Is a MEX file.
% - The structuring element should have an odd side length.
% - Is slow since the function FUNC must be invoked once for every 
%   output pixel.
% - The input can be logical, uint8, uint16, float or double, the output is
%   always double
%
% See also IVAR, IRANK.



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

if ~exist('iwindow', 'file')
    error('you need to build the MEX version of iwindow, see vision/mex/README');
end
