%ISCALESPACE  Scale-space image sequence
%
% [G,L,S] = ISCALESPACE(IM, N, SIGMA) is a scale space image sequence of 
% length N derived from IM (HxW).  The standard deviation of the smoothing 
% Gaussian is SIGMA.  At each scale step the variance of the Gaussian increases
% by SIGMA^2.  The first step in the sequence is the original image.
%
% G (HxWxN) is the scale sequence, L (HxWxN) is the absolute value of the 
% Laplacian of Gaussian (LoG) of the scale sequence, corresponding to each 
% step of the sequence, and S (Nx1) is the vector of scales.
%
% [G,L,S] = ISCALESPACE(IM, N) as above but SIGMA=1.
%
% Examples::
%  Create a scale-space image sequence
%         im = iread('lena.png', 'double', 'grey');
%         [G,L,s] = iscalespace(im, 50, 2);
%  Then find scale-space maxima, an array of ScalePointFeature objects.
%         f = iscalemax(L, s);
%  Look at the scalespace volume
%         slice(L, [], [], 5:10:50); shading interp
%
% Notes::
% - The Laplacian is approximated by the the difference of adjacent Gaussians.
%
% See also ISCALEMAX, ISMOOTH, ILAPLACE, KLOG.


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

function [G, L, S] = scalespace(im, n, sigma)

    if nargin < 3
        sigma = 1;
    end

    g(:,:,1) = im;
    scale = 0.5;
    scales = scale;
    for i=1:n-1,
        im = ismooth(im, sigma);
        scale = sqrt(scale^2 + sigma^2);
        scales = [scales; scale];
        g(:,:,i+1) = im;
        lap(:,:,i) = scale^2 * ( g(:,:,i+1) - g(:,:,i) );
    end

    % return results as requested
    if nargout > 2,
        S = scales;
    end
    if nargout > 0,
        G = g;
    end
    if nargout > 1,
        L = lap;
    end
