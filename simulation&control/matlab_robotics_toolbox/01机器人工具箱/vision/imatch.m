%IMATCH Template matching
%
% XM = IMATCH(IM1, IM2, U, V, H, S) is the position of the matching subimage of
% IM1 (template) within the image IM2.  The template in IM1 is centred at (U,V)
% and its half-width is H.  
%
% The template is searched for within IM2 inside a rectangular region, centred 
% at (U,V) and whose size is a function of S.  If S is a scalar the search 
% region is [-S, S, -S, S] relative to (U,V).  More generally S is a 4-vector 
% S=[umin, umax, vmin, vmax] relative to (U,V).
%
% The return value is XM=[DU,DV,CC] where (DU,DV) are the u- and v-offsets 
% relative to (U,V) and CC is the similarity score for the best match in the
% search region.
%
% [XM,SCORE] = IMATCH(IM1, IM2, U, V, H, S) as above but also returns a matrix
% of matching score values for each template position tested. The rows 
% correspond to horizontal positions of the template, and columns the vertical
% position.  The centre element corresponds to (U,V).
%
% Example::
%  Consider a sequence of images im(:,:,N) and we find corner points in the
%  k'th image
%        corners = icorner(im(:,:,k), 'nfeat', 20);
%  Now, for each corner we look for the 11x11 patch of surrounding pixels
%  in the next image, by searching within a 21x21 region
%        for corner=corners
%           xm = imatch(im(:,:,k), im(:,:,k+1), 5, 10);
%           if xm(3) > 0.8
%               fprintf('feature (%f,%f) moved by (%f,%f) pixels)\n', ...
%                   corner.u, corner.v, xm(1), xm(2) );
%           end
%        end
%
% Notes::
% - Useful for tracking a template in an image sequence where IM1 and IM2
%   are consecutive images in a template and (U,V) is the coordinate of
%   a corner point in IM1.
% - Is a MEX file.
% - IM1 and IM2 must be the same size.
% - ZNCC (zero-mean normalized cross correlation) matching is used as the 
%   similarity measure. A perfect match score is 1.0 but anything above 0.8
%   is typically considered to be a good match.
%
% See also ISIMILARITY.


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

if ~exist('imatch', 'file')
    error('you need to build the MEX version of imatch, see vision/mex/README');
end
