%KDGAUSS Derivative of Gaussian kernel
%
% K = KDGAUSS(SIGMA) is a 2-dimensional derivative of Gaussian kernel (WxW)
% of width (standard deviation) SIGMA and centred within the matrix K whose 
% half-width H = 3xSIGMA and W=2xH+1.
%
% K = KDGAUSS(SIGMA, H) as above but the half-width is explictly specified.
%
% Notes::
% - This kernel is the horizontal derivative of the Gaussian, dG/dx.
% - The vertical derivative, dG/dy, is K'.
% - This kernel is an effective edge detector.
%
% See also KGAUSS, KDOG, KLOG, ISOBEL, ICONV.



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

function m = dgauss(sigma, w)


    if nargin == 1,
        w = ceil(3*sigma);
    end
    ww = 2*w + 1;

    [x,y] = meshgrid(-w:w, -w:w);

    m = -x/sigma^2 /(2*pi) .*  exp( -(x.^2 + y.^2)/2/sigma^2);

    %m = m / sum(sum(m));

