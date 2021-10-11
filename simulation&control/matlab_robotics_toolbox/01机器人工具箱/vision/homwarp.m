%HOMWARP Warp image by an homography
%
% OUT = HOMWARP(H, IM, OPTIONS) is a warp of the image IM obtained by 
% applying the homography H to the coordinates of every input pixel.
%
% [OUT,OFFS] = HOMWARP(H, IM, OPTIONS) as above but OFFS is the offset of the
% warped tile OUT with respect to the origin of IM.
%
% Options::
% 'full'            output image contains all the warped pixels, but its
%                   position with respect to the input image is given by the
%                   second return value OFFS.
% 'extrapval',V     set unmapped pixels to this value (default NaN)
% 'roi',R           output image contains the specified ROI in the input image
% 'scale',S         scale the output by this factor
% 'dimension',D     ensure output image is DxD
% 'size',S          size of output image S=[W,H]
% 'coords',{U,V}    coordinate matrices for im, each same size as im.
%
% Notes::
% - The edges of the resulting output image will in general not be
%   be vertical and horizontal lines.
%
% See also HOMOGRAPHY, ITRIM, INTERP2.


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

function [w,foffs] = homwarp(H, im, varargin)

    % the result of a warp can have negative pixel coordinates or very
    % large coordinates, well away from the origin.

    opt.full = false;
    opt.extrapval = NaN;
    opt.size = [];
    opt.roi = [];
    opt.scale = 1;
    opt.dimension = [];
    opt.coords = [];

    opt = tb_optparse(opt, varargin);

    [w,h] = isize(im);

    if opt.roi
        % opt.box is specified in standard ROI format
        l = opt.roi(1,1); t = opt.roi(2,1);
        r = opt.roi(1,2); b = opt.roi(2,2);
        box = [l r r l; t t b b];
    else
        % bounding box in the input image is the full extent
        box = [1 w w 1; 1 1 h h];
    end

    % map the box vertices in input image to vertices in output image
    Hbox = homtrans(H, box);

    % determine the extent of the image after warping
    xmin = min(Hbox(1,:)); xmax = max(Hbox(1,:));
    ymin = min(Hbox(2,:)); ymax = max(Hbox(2,:));

    % we want the pixel coordinates to map to positive values, determine the minimum
    if opt.full
        offs = floor([xmin, ymin]);

        % and prepend a translational homography that translates the output image
        H = [1 0 -offs(1); 0 1 -offs(2); 0 0 1] * H;
    end
    
    sz = round([xmax-xmin+1, ymax-ymin+1]);

    % we can specify the maxmimum dimension of the resulting image
    if ~isempty(opt.dimension)
        s = opt.dimension / max(sz);
        H = diag([s s 1]) * H;

        Hbox = homtrans(H, box);

        % determine the extent
        xmin = min(Hbox(1,:)); xmax = max(Hbox(1,:));
        ymin = min(Hbox(2,:)); ymax = max(Hbox(2,:));

        % we want the pixel coordinates to map to positive values, determine the minimum
        offs = floor([xmin, ymin]);

        % and prepend a translational homography that translates the output image
        H = [1 0 -offs(1); 0 1 -offs(2); 0 0 1] * H;
        sz = round([xmax-xmin+1, ymax-ymin+1]);
    end
    
    if isempty(opt.coords)
        [Ui,Vi] = imeshgrid(im);
    else
        Ui = opt.coords{1};
        Vi = opt.coords{2};
    end

    % determine size of the output image
    if ~isempty(opt.size)
        [Uo,Vo] = imeshgrid(opt.size);
    else
        if opt.full
            [Uo,Vo] = imeshgrid(sz);
        else
            [Uo,Vo] = imeshgrid(im);
        end
    end
    
    % warp the coordinates of the output pixels    
    UV = homtrans(inv(H), [Uo(:) Vo(:)]');
    U = reshape(UV(1,:), size(Uo));
    V = reshape(UV(2,:), size(Vo));

    % interpolate for each color plane
    for p=1:size(im,3)
        imh(:,:,p) = interp2(Ui, Vi, idouble(im(:,:,p)), U, V, 'linear', opt.extrapval);
    end

    if nargout > 0
        w = imh;
    else
        idisp(imh);
    end

    if nargout > 1 && opt.full
        foffs = offs;
    end
