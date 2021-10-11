%COLORKMEANS Color image segmentation by clustering
%
% L = COLORKMEANS(IM, K, OPTIONS) is a segmentation of the color image IM 
% into K classes.  The label image L has the same row and column dimension
% as IM and each pixel has a value in the range 0 to K-1 which indicates
% which cluster the corresponding pixel belongs to.  A k-means clustering of
% the chromaticity of all input pixels is performed.
%
% [L,C] = COLORKMEANS(IM, K) as above but also returns the cluster 
% centres C (Kx2) where the I'th row is the rg-chromaticity of the I'th
% cluster and corresponds to the label I.  A k-means clustering of the 
% chromaticity of all input pixels is performed.
%
% [L,C,R] = COLORKMEANS(IM, K) as above but also returns the residual R, the 
% root mean square error of all pixel chromaticities with respect to their 
% cluster centre.
%
% L = COLORKMEANS(IM, C) is a segmentation of the color image IM into K classes
% which are defined by the cluster centres C (Kx2) in chromaticity space.
% Pixels are assigned to the closest (Euclidean) centre.  Since cluster 
% centres are provided the k-means segmentation step is not required.
%
% Options::
%
% Various options are possible to choose the initial cluster centres 
% for k-means:
% 'random'   randomly choose K points from
% 'spread'   randomly choose K values within the rectangle spanned by the
%            input chromaticities.
% 'pick'     interactively pick cluster centres
%
% Notes::
% - The k-means clustering algorithm used in the first three forms is
%   computationally expensive and time consuming.
% - Clustering is performed in xy-chromaticity space.
% - The residual is an indication of quality of fit, low is good.
%
% See also RGB2XYZ, KMEANS.



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
function [labels,C,resid] = colorkmeans(im, k, varargin)

    % convert RGB to xy space
    rgbcol = im2col(im);
    XYZcol = rgb2xyz(rgbcol);
    sXYZ = sum(XYZcol')';
    x = XYZcol(:,1) ./ sXYZ;
    y = XYZcol(:,2) ./ sXYZ;
    
    if any(isnan(x)) || any(isnan(y))
        error('undefined xy chromaticity for some pixels: input image has pixels with value (0,0,0)');
    end
    
    % do the k-means clustering
    
    if numcols(k) > 1 && numrows(k) == 2
        % k is cluster centres
        [L,C,resid] = kmeans([x y]', k, varargin{:});
    else
        if length(varargin) > 0 && strcmp(varargin{1}, 'pick')
                z0 = pickpoints(k, im, x, y);
                [L,C,resid] = kmeans([x y]', k, z0', varargin{:});
        else
            [L,C,resid] = kmeans([x y]', k, varargin{:});
        end
    end
    
    % convert labels back to an image
    L = col2im(L', im);
        
    for k=1:numrows(C)
        fprintf('%2d: ', k);
        fprintf('%11.4g ', C(k,:));
        fprintf('\n');
        %fprintf('%s\n', colorname(C(k,:)));
    end
    
    if nargout > 0
        labels = L;
    end
end
    
function z0 = pickpoints(k, im, x, y)

    fprintf('Select %d points to serve as cluster centres\n', k);
    clf
    image(im)
    uv = round( ginput(k) );
    sz = size(im);
    i = sub2ind( sz(1:2), uv(:,2), uv(:,1) );
    
    z0 =[x(i) y(i)];
end
