%COLORDISTANCE Colorspace distance
%
% D = COLORDISTANCE(IM, RG) is the Euclidean distance on the rg-chromaticity 
% plane from coordinate RG=[r,g] to every pixel in the color image IM.  D is 
% an image with the same dimensions as IM and the value of each pixel is 
% the color space distance of the corresponding pixel in IM.
%
% Notes::
% - The output image could be thresholded to determine color similarity.
% - Note that Euclidean distance in the rg-chromaticity space does not 
%   correspond well with human perception of color differences.  Perceptually
%   uniform spaces such as Lab remedy this problem.
%
% See also COLORSPACE.


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

% Peter Corke 2005
%

function s = colordistance(im, rg)

    % convert image to (r,g) coordinates
    rgb = sum(im, 3);
    r = im(:,:,1) ./ rgb;
    g = im(:,:,2) ./ rgb;

    % compute the Euclidean color space distance
    d = (r - rg(1)).^2 + (g - rg(2)).^2;

    if nargout == 0,
        idisp(d)
    else
        s = d;
    end

    


