%KDOG Difference of Gaussian kernel
%
% K = KDOG(SIGMA1) is a 2-dimensional difference of Gaussian kernel equal 
% to KGAUSS(SIGMA1) - KGAUSS(SIGMA2), where SIGMA1 > SIGMA2.  By default
% SIGMA2 = 1.6*SIGMA1.  The kernel is centred within the matrix K whose 
% half-width H = 3xSIGMA and W=2xH+1.
% 
% K = KDOG(SIGMA1, SIGMA2) as above but SIGMA2 is specified directly.
%
% K = KDOG(SIGMA1, SIGMA2, H) as above but the kernel half-width is specified.
%
% Notes::
% - This kernel is similar to the Laplacian of Gaussian and is often used
%   as an efficient approximation.
%
% See also KGAUSS, KDGAUSS, KLOG, ICONV.



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

function m = kdog(sigma1, sigma2, w)

    % sigma1 > sigma2
    if nargin == 1
        sigma2 = 1.6*sigma1;
        w = ceil(2*sigma2);
    elseif nargin == 2
        sigma2 = 1.6*sigma1;
        w = ceil(2*sigma2);
    elseif nargin == 3
        if sigma2 < sigma1
            t = sigma1;
            sigma1 = sigma2;
            sigma2 = t;
        end
    end
	if nargin < 3
        w = ceil(3*sigma1);
	end

    % sigma2 > sigma1
    m1 = kgauss(sigma1, w);     % thin kernel
    m2 = kgauss(sigma2, w);     % wide kernel

    m = m2 - m1;
