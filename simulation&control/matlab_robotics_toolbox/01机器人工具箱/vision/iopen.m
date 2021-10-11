%IOPEN Morphological opening
%
% OUT = IOPEN(IM, SE, OPTIONS) is the image IM after morphological opening 
% with the structuring element SE.  This is a morphological erosion followed 
% by dilation.
%
% OUT = IOPEN(IM, SE, N, OPTIONS) as above but the structuring element 
% SE is applied N times, that is N erosions followed by N dilations.
%
% Notes::
% - For binary image an opening operation can be used to eliminate small white
%   noise regions.
% - It is cheaper to apply a smaller structuring element multiple times than
%   one large one, the effective structuring element is the Minkowski sum
%   of the structuring element with itself N times.
% - Windowing options of IMORPH can be passed.  By default output image is
%   same size as input image.
%
% See also ICLOSE, IDILATE, IERODE, IMORPH.

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

function b = iopen(a, se, varargin)

    b = ierode(a, se, varargin{:});
    b = idilate(b, se, varargin{:});
