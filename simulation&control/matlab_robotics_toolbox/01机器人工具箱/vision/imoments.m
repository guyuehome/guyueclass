%IMOMENTS Image moments
%
% F = IMOMENTS(IM) is a RegionFeature object that describes the greyscale
% moments of the image IM.  
%
% F = IMOMENTS(U, V) as above but the moments are computed from the pixel 
% coordinates given as vectors U (Nx1) and V (Nx1).  All pixels are equally
% weighted and is effectively a binary image.
%
% F = IMOMENTS(U, V, W) as above but the pixels have weights given by the
% vector W and is effectively a greyscale image.
%
% Properties::
% The RegionFeature object has many properties including:
%
%  uc        centroid, horizontal coordinate
%  vc        centroid, vertical coordinate
%  area      the number of pixels
%  a         major axis length of equivalent ellipse
%  b         minor axis length of equivalent ellipse
%  theta     angle of major ellipse axis to horizontal axis
%  shape     aspect ratio b/a (always <= 1.0)
%  moments   a structure containing moments of order 0 to 2, the elements
%            are m00, m10, m01, m20, m02, m11.
%
% See RegionFeature help for more details.
%
% Notes::
% - For a binary image the zeroth moment is the number of non-zero pixels, or
%   its area.
% - This function does not perform connectivity it considers all non-zero
%   pixels in the image.  If connected regions are required then use IBLOBS
%   instead.
%
% See also RegionFeature, IBLOBS.



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

function F = imoments(varargin)

    opt.aspect = 1;
    
    [opt,args] = tb_optparse(opt, varargin);
    
    if length(args) == 1
        im = args{1};
        [v,u] = find(im);
        w = im(find(im > 0));
    else
        u = args{1};
        v = args{2};
        if length(args) == 3
            w = args{3};
        else
            w = ones(size(u));
        end
    end

    % compute basic moments
    m00 = sum(w);
    m10 = sum(u.*w);
    m01 = sum(v.*w*opt.aspect);
    m20 = sum((u.^2).*w);
    m02 = sum((v.^2).*w*opt.aspect^2);
    m11 = sum((u.*v).*w*opt.aspect);

    if m00 > 0
        % figure some central moments
        u20 = m20 - m10^2/m00;
        u02 = m02 - m01^2/m00;
        u11 = m11 - m10*m01/m00;

        % figure the equivalent axis lengths, function of the principal axis lengths
        [x,e] = eig([u20 u11; u11 u02]);
        a = 2*sqrt(max(diag(e))/m00);
        b = 2*sqrt(min(diag(e))/m00);
        v = x(:,end);
        th = atan2(v(2),v(1));

        %th = 0.5*atan2(2*u11, u20-u02);
    else
        u20 = NaN;
        u02 = NaN;
        u11 = NaN;

        a = NaN;
        b = NaN;
        th = NaN;
    end

    %F = [m00 m10/m00 m01/m00 a b th];
    F = RegionFeature;
    F.area_ = m00;
    if m00 > 0
        F.uc_ = m10/m00;
        F.vc_ = m01/m00;
    else
        F.uc_ = NaN;
        F.vc_ = NaN;
    end
    F.a = a;
    F.b = b;
    F.shape_ = b/a;
    F.theta_ = th;
    F.moments.m00 = m00;
    F.moments.m01 = m01;
    F.moments.m10 = m10;
    F.moments.m02 = m02;
    F.moments.m20 = m20;
    F.moments.m11 = m11;
    F.moments.u20 = u20;
    F.moments.u02 = u02;
    F.moments.u11 = u11;
