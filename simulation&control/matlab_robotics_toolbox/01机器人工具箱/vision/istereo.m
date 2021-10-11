%ISTEREO Stereo matching
%
% D = ISTEREO(LEFT, RIGHT, RANGE, H, OPTIONS) is a disparity image computed
% from the epipolar aligned stereo pair: the left image LEFT (HxW) and the
% right image RIGHT (HxW).  D (HxW) is the disparity and the value at each 
% pixel is the horizontal shift of the corresponding pixel in IML as observed 
% in IMR. That is, the disparity d=D(v,u) means that the pixel at RIGHT(v,u-d)
% is the same world point as the pixel at LEFT(v,u).
%
% RANGE is the disparity search range, which can be a scalar for disparities in
% the range 0 to RANGE, or a 2-vector [DMIN DMAX] for searches in the range
% DMIN to DMAX.
%
% H is the half size of the matching window, which can be a scalar for NxN or a
% 2-vector [N,M] for an NxM window.
%
% [D,SIM] = ISTEREO(LEFT, RIGHT, RANGE, H, OPTIONS) as above but returns SIM 
% which is the same size as D and the elements are the peak matching score 
% for the corresponding elements of D.  For the default matching metric ZNCC
% this varies between -1 (very bad) to +1 (perfect).
%
% [D,SIM,DSI] = ISTEREO(LEFT, RIGHT, RANGE, H, OPTIONS) as above but returns DSI 
% which is the disparity space image (HxWxN) where N=DMAX-DMIN+1. The I'th 
% plane is the similarity of IML to IMR shifted to the left by DMIN+I-1.
%
% [D,SIM,P] = ISTEREO(LEFT, RIGHT, RANGE, H, OPTIONS) if the 'interp' option is 
% given then disparity is estimated to sub-pixel precision using quadratic
% interpolation.  In this case D is the interpolated disparity and P is
% a structure with elements A, B, dx.  The interpolation polynomial is 
% s = Ad^2 + Bd + C where s is the similarity score and d is disparity relative
% to the integer disparity at which s is maximum.  P.A and P.B are matrices the
% same size as D whose elements are the per pixel values of the interpolation
% polynomial coefficients.  P.dx is the peak of the polynomial with respect
% to the integer disparity at which s is maximum (in the range -0.5 to +0.5).
%   
% Options::
% 'metric',M   string that specifies the similarity metric to use which is
%              one of 'zncc' (default), 'ncc', 'ssd' or 'sad'.
% 'interp'     enable subpixel interpolation and D contains non-integer
%              values (default false)
% 'vshift',V   move the right image V pixels vertically with respect to left.
%
% Example::
%
% Load the left and right images
%         L = iread('rocks2-l.png', 'reduce', 2);
%         R = iread('rocks2-r.png', 'reduce', 2);
% then compute stereo disparity and display it
%         d = istereo(L, R, [40, 90], 3);
%         idisp(d);
%
% References::
%  - Robotics, Vision & Control, Section 14.3,
%    P. Corke, Springer 2011.
%
% Notes::
% - Images must be greyscale.
% - Disparity values pixels within a half-window dimension (H) of the edges 
%   will not be valid and are set to NaN.
% - The C term of the interpolation polynomial is not computed or returned.
% - The A term is high where the disparity function has a sharp peak.
% - Disparity and similarity score can be obtained from the disparity space
%   image by [SIM,D] = max(DSI, [], 3)
%
%
% See also IRECTIFY, STDISP.




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

function [disp,sim, o3] = istereo(L, R, drange, h, varargin)

% TODO: score cube is float, can return it

    opt.metric = 'zncc';
    opt.interp = false;
    opt.vshift = 0;

    opt = tb_optparse(opt, varargin);

    % ensure images are greyscale
    L = imono(L);
    R = imono(R);

    opt.vshift = round(opt.vshift);
    if opt.vshift ~= 0
        if opt.vshift > 0
            L = L(1:end-opt.vshift,:);
            R = R(opt.vshift:end,:);
        else
            vshift = -vshift;
            L = L(opt.vshift:end,:);
            R = R(1:end-opt.vshift,:);
        end
    end
        
    % compute the score cube, 3rd dimension is disparity
    DSI = stereo_match(L, R, 2*h+1, drange(1:2), opt.metric);

    % best value along disparity dimension is the peak
    %   s best score
    %   d disparity at which it occurs
    %
    % both s and d are matrices same size as L and R.
    if strcmp(opt.metric, 'sad') | strcmp(opt.metric, 'ssd')
        [s,d] = min(DSI, [], 3);
    else
        [s,d] = max(DSI, [], 3);
    end

    d(isnan(s)) = NaN;


    if opt.interp
        % interpolated result required

        % get number of pixels and disparity range
        npix = prod(size(L));

        if length(drange) == 1,
            ndisp = drange + 1;
        else
            dmin = min(drange);
            dmax = max(drange);
            ndisp = dmax - dmin + 1;
        end

        % find all disparities that are not at either end of the range, we need
        % a point on either side to interpolate them
        valid = (d>1) & (d<ndisp);
        valid = valid(:);

        % make a vector of consecutive pixel indices (1 to width*height)
        ci = [1:npix]';
        % turn disparities into a column vector
        dcol = d(:);

        % remove all entries that are not valid
        ci(~valid) = [];
        dcol(~valid) = [];

        % both ci and dcol have the same number of entries

        % for every valid pixel and disparity, find the index into the 3D score
        % array.  We cheat and consider that array WxHxD as a 2D array (WxH)xD
        %
        % We compute the indices for the best score and one each side of it
        k_m = sub2ind([npix ndisp], ci, dcol-1);
        k_0 = sub2ind([npix ndisp], ci, dcol);
        k_p = sub2ind([npix ndisp], ci, dcol+1);

        % initialize matrices (size of L and R) to hold the the best score
        % and the one each side of it
        y_m = ones(size(L))*NaN;
        y_0 = ones(size(L))*NaN;
        y_p = ones(size(L))*NaN;

        % now copy over the valid scores into these arrays.  What doesnt
        % get copies is a NaN
        y_m(ci) = DSI(k_m);
        y_0(ci) = DSI(k_0);
        y_p(ci) = DSI(k_p);

        % figure the coefficients of the peak fitting parabola:
        %    y = Ax^2 + Bx + C
        % Each coefficient is a matrix same size as (L and R)
        % We don't need to compute C
        A = 0.5*y_m - y_0 + 0.5*y_p;
        B = -0.5*y_m + 0.5*y_p;

        % now the position of the peak is given by -B/2A
        dx = -B ./ (2*A);

        % and we add this fractional part to the integer value obtained
        % from the max/min function
        d = d + dx;

   end
   d = d + drange(1)-1;

    if nargout > 0,
        disp = d;
    end
    if nargout > 1,
        sim = s;
    end

    if nargout > 2
        if opt.interp
            o3.A = A;
            o3.B = B;
            o3.dx = dx;
        else
            o3 = DSI;
        end
    end
