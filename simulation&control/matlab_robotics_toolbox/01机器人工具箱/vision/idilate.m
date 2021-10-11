%IDILATE Morphological dilation
%
% OUT = IDILATE(IM, SE, OPTIONS) is the image IM after morphological dilation 
% with the structuring element SE.
%
% OUT = IDILATE(IM, SE, N, OPTIONS) as above but the structuring element 
% SE is applied N times, that is N dilations.
%
% Options::
% 'border'   the border value is replicated (default)
% 'none'     pixels beyond the border are not included in the window
% 'trim'     output is not computed for pixels where the structuring element
%            crosses the image border, hence output image had reduced 
%            dimensions.
% 'wrap'     the image is assumed to wrap around, left to right, top to
%            bottom.
% 
% Notes::
% - Cheaper to apply a smaller structuring element multiple times than
%   one large one, the effective structuring element is the Minkowski sum
%   of the structuring element with itself N times.
% - Windowing options of IMORPH can be passed.
%
% Reference::
%  - Robotics, Vision & Control, Section 12.5,
%    P. Corke, Springer 2011.
%
% See also IERODE, ICLOSE, IOPEN, IMORPH.

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
function dilated = idilate(im, se, varargin)

    if nargin > 2 && ~ischar(varargin{1})
        n = varargin{1};
        varargin = varargin(2:end);
    else
        n = 1;
    end

    dilated = im;
    for i=1:n
        dilated = imorph(dilated, se, 'max', varargin{:});
    end
