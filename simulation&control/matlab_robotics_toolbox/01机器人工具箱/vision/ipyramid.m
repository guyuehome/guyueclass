%IPYRAMID Pyramidal image decomposition
%
% OUT = IPYRAMID(IM) is a pyramid decomposition of input image IM using 
% Gaussian smoothing with standard deviation of 1.  OUT is a cell array of
% images each one having dimensions half that of the previous image. The 
% pyramid is computed down to a non-halvable image size.
%
% OUT = IPYRAMID(IM, SIGMA) as above but the Gaussian standard deviation 
% is SIGMA.
%
% OUT = IPYRAMID(IM, SIGMA, N) as above but only N levels of the pyramid are
% computed.
%
% Notes::
% - Works for greyscale images only.
%
% See also ISCALESPACE, IDECIMATE, ISMOOTH.



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

function p = ipyramid(im, sigma, N)
    if iscolor(im)
        error('greyscale images only');
    end
    if nargin < 2,
        sigma = 1;
        N = floor( log2( min(size(im) ) ) );
    elseif nargin < 3,
        N = floor(log2(min(size(im))));
    end

    [height,width] = size(im);
    K = kgauss(sigma);

    p{1} = im;

    for k = 1:N,
        [nrows,ncols] = size(im);

        % smooth
        im = conv2(im, K, 'same');

        % sub sample
        im = im(1:2:nrows,1:2:ncols);

        % stash it
        p{k+1} = im;
    end
