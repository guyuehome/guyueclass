%IDECIMATE	Decimate an image
%
% S = IDECIMATE(IM, M) is a decimated version of the image IM whose
% size is reduced by M (an integer) in both dimensions.  The image is smoothed
% with a Gaussian kernel with standard deviation M/2 then subsampled.
%
% S = IDECIMATE(IM, M, SD) as above but the standard deviation of the
% smoothing kernel is set to SD.
%
% S = IDECIMATE(IM, M, []) as above but no smoothing is applied prior
% to decimation.
%
% Notes::
% - If the image has multiple planes, each plane is decimated.
% - Smoothing is used to eliminate aliasing artifacts and the standard 
%   deviation should be chosen as a function of the maximum spatial frequency
%   in the image.
%
% See also ISCALE, ISMOOTH, IREPLICATE.



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

function s = shrink(im, m, sigma)

    if nargin < 2
        m = 2;
    end

    if (m - ceil(m)) ~= 0
        error('decimation factor must be integer');
    end

    if nargin < 3
        sigma = m / 2;
    end

    % smooth the image
    if ~isempty(sigma)
        im = ismooth(im, sigma);
    end

    % then decimate
	s = im(1:m:end,1:m:end,:);
