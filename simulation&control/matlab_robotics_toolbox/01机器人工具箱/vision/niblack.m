%NIBLACK Adaptive thresholding
%
% T = NIBLACK(IM, K, W2) is the per-pixel (local) threshold to apply to 
% image IM.  T has the same dimensions as IM.  The threshold at each pixel is 
% a function of the mean and standard deviation computed over a WxW window,
% where W=2*w2+1.
%
% [T,M,S] = NIBLACK(IM, K, W2) as above but returns the per-pixel mean M
% and standard deviation S.
%
% Example::
%     t = niblack(im, -0.2, 20);
%     idisp(im >= t);
%
% Notes::
% - This is an efficient algorithm very well suited for binarizing
%   text.
% - W2 should be chosen to be half the "size" of the features to be 
%   segmented, for example, in text segmentation, the height of a 
%   character.
% - A common choice of k=-0.2
%
%
% Reference::
% An Introduction to Digital Image Processing,
% W. Niblack, 
% Prentice-Hall, 1986.
%
% See also OTSU, ITHRESH.


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

function [t,M,S] = niblack(im, k, w2)

    if nargin < 3,
        w2 = 7;
    end
    w = 2*w2 + 1;

    window = ones(w, w);

    % compute sum of pixels in WxW window
    sp = conv2(im, window, 'same');
    % convert to mean
    n = w^2;            % number of pixels in window
    m = sp / n;

    if k ~= 0
        % compute sum of pixels squared in WxW window
        sp2 = conv2(im.^2, window, 'same');
        % convert to std
        var = (n*sp2 - sp.^2) / n / (n-1);
        s = sqrt(var);

        % compute Niblack threshold
        t = m + k * s;
    else
        t = m;
        s = [];
    end

    if nargout > 1
        M = m;
    end
    if nargout > 2
        S = s;
    end
