%IVAR Pixel window statistics
%
% OUT = IVAR(IM, SE, OP) is an image where each output pixel is
% the specified statistic over the pixel neighbourhood indicated by the
% structuring element SE which should have odd side lengths.  The elements 
% in the neighbourhood corresponding to non-zero elements in SE are packed into
% a vector on which the required statistic is computed.
%
% The operation OP is one of:
% 'var'    variance
% 'kurt'   Kurtosis or peakiness of the distribution
% 'skew'   skew or asymmetry of the distribution
%
% OUT = IVAR(IM, SE, OP, EDGE) as above but performance at edge pixels
% can be controlled.  The value of EDGE is:
% 'border'   the border value is replicated (default)
% 'none'     pixels beyond the border are not included in the window
% 'trim'     output is not computed for pixels whose window crosses
%            the border, hence output image had reduced dimensions.
% 'wrap'     the image is assumed to wrap around
%
% Notes::
% - Is a MEX file.
% - The structuring element should have an odd side length.
% - The input can be logical, uint8, uint16, float or double, the output is
%   always double
%
% See also IRANK, IWINDOW.



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

if ~exist('ivar', 'file')
    error('you need to build the MEX version of ivar, see vision/mex/README');
end
