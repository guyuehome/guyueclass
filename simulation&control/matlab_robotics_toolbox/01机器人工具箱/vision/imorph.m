%IMORPH Morphological neighbourhood processing
%
% OUT = IMORPH(IM, SE, OP) is the image IM after morphological processing 
% with the operator OP and structuring element SE.
%
% The structuring element SE is a small matrix with binary values that indicate
% which elements of the template window are used in the operation. 
%
% The operation OP is:
% 'min'       minimum value over the structuring element
% 'max'       maximum value over the structuring element
% 'diff'      maximum - minimum value over the structuring element
% 'plusmin'   the minimum of the pixel value and the pixelwise sum of the
%             structuring element and source neighbourhood.
%
% OUT = IMORPH(IM, SE, OP, EDGE) as above but performance of edge pixels
% can be controlled.  The value of EDGE is:
% 'border'   the border value is replicated (default)
% 'none'     pixels beyond the border are not included in the window
% 'trim'     output is not computed for pixels where the structuring element
%            crosses the image border, hence output image had reduced 
%            dimensions.
% 'wrap'     the image is assumed to wrap around, left to right, top to
%            bottom.
%
% Notes::
% - Is a MEX file.
% - Performs greyscale morphology.
% - The structuring element should have an odd side length.
% - For binary image 'min' = EROSION, 'max' = DILATION.
% - The 'plusmin' operation can be used to compute the distance transform.
% - The input can be logical, uint8, uint16, float or double, the output is
%   always double
%
% See also IRANK, IVAR, HITORMISS, IOPEN, ICLOSE, DTRANSFORM.



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
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

if ~exist('imorph', 'file')
    error('you need to build the MEX version of imorph, see vision/mex/README');
end
