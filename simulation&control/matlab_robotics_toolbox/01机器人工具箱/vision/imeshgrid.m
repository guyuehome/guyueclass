%IMESHGRID Domain matrices for image
%
% [U,V] = IMESHGRID(IM) are matrices that describe the domain of image IM (HxW)
% and are each HxW.  These matrices are used for the evaluation of functions 
% over the image. The element U(R,C) = C and V(R,C) = R.
%
% [U,V] = IMESHGRID(W, H) as above but the domain is WxH.
%
% [U,V] = IMESHGRID(S) as above but the domain is described by S which can
% be a scalar SxS or a 2-vector S=[W, H].
%
% See also MESHGRID, INTERP2.


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

function [U,V] = imeshgrid(a1, a2)

    if nargin == 1
        if length(a1) == 1
            % we specified a size for a square output image
            [U,V] = meshgrid(1:a1, 1:a1);
        elseif length(a1) == 2
            % we specified a size for a rectangular output image (w,h)
            [U,V] = meshgrid(1:a1(1), 1:a1(2));
        elseif ndims(a1) >= 2
            [U,V] = meshgrid(1:numcols(a1), 1:numrows(a1));
        else
            error('incorrect argument');
        end
    elseif nargin == 2
        [U,V] = meshgrid(1:a1, 1:a2);
    end
        
        
