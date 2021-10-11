%ISAMESIZE Automatic image trimming
%
% OUT = ISAMESIZE(IM1, IM2) is an image derived from IM1 that has
% the same dimensions as IM2.  This is achieved by cropping and scaling.
%
% OUT = ISAMESIZE(IM1, IM2, BIAS) as above but BIAS controls which part
% of the image is cropped.  BIAS=0.5 is symmetric cropping, BIAS<0.5 moves
% the crop window up or to the left, while BIAS>0.5 moves the crop window
% down or to the right.
%
% See also ISCALE, IROI, ITRIM.


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

function im2 = isamesize(im, im1, bias)
    % return im scaled/cropped to be the same size as im1

    if nargin < 3
        bias = 0.5;
    end

    if bias < 0 || bias > 1
        error('bias must be in the range [0,1]')
    end
    sz1 = size(im1);
    sz = size(im);
    scale = sz1(1:2) ./ sz(1:2);

    scale
    im2 = iscale(im, max(scale));

    if numrows(im2) > numrows(im1)
        % scaled image is too high, trim some rows
        d = numrows(im2) - numrows(im1);
        d1 = max(1, floor(d*bias));
        d2 = d-d1;
        [1 d d1 d2]
        im2 = im2(d1:end-d2-1,:,:);
    end
    if numcols(im2) > numcols(im1)
        % scaled image is too wide, trim some columns
        d = numcols(im2) - numcols(im1);
        d1 = max(1, floor(d*bias));
        d2 = d-d1;
        [2 d d1 d2]
        im2 = im2(:,d1:end-d2-1,:);
    end
