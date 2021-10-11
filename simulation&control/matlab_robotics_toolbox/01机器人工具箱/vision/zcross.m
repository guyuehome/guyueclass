%ZCROSS Zero-crossing detector
%
% IZ = ZCROSS(IM) is a binary image with pixels set where the corresponding 
% pixels in the signed image IM have a zero crossing, a positive pixel 
% adjacent to a negative pixel.
%
% Notes::
% - Can be used in association with a Lapalacian of Gaussian image to 
%   determine edges.
%
% See also ILOG.



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

function iz = zcross(im)

    z = zeros([size(im), 4]);
    K = [1 1 0 ;1 1 0; 0 0 0];
    z(:,:,1) = conv2(im, K, 'same');
    K = [0 1 1 ;0 1 1; 0 0 0];
    z(:,:,2) = conv2(im, K, 'same');
    K = [0 0 0; 1 1 0 ;1 1 0];
    z(:,:,3) = conv2(im, K, 'same');
    K = [0 0 0; 0 1 1 ;0 1 1];
    z(:,:,4) = conv2(im, K, 'same');

    maxval = max(z, [], 3);
    minval = min(z, [], 3);

    iz = (maxval > 0) & (minval < 0);
