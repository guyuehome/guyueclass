%IROTATE Rotate image
%
% OUT = IROTATE(IM, ANGLE, OPTIONS) is a version of the image IM
% that has been rotated about its centre.
%
% Options::
% 'outsize',S     set size of output image to HxW where S=[W,H]
% 'crop'          return central part of image, same size as IM
% 'scale',S       scale the image size by S (default 1)
% 'extrapval',V   set background pixels to V (default 0)
% 'smooth',S      initially smooth the image with a Gaussian of standard 
%                 deviation S
%
% Notes::
% - Rotation is defined with respect to a z-axis which is into the image.
% - Counter-clockwise is a positive angle.
% - The pixels in the corners of the resulting image will be undefined and
%   set to the 'extrapval'.
%
% See also ISCALE.


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

function im2 = irotate(im, angle, varargin)

    opt.outsize = [];
    opt.crop = false;
    opt.scale = 1.0;
    opt.extrapval = 0;
    opt.smooth = [];
    
    opt = tb_optparse(opt, varargin);

    if isfloat(im)
        is_int = false;
    else
        is_int = true;
        im = idouble(im);
    end

    if ~isempty(opt.smooth)
        im = ismooth(im, opt.smooth);
    end

    [nr,nc,np] = size(im);

    if isempty(opt.outsize)
        % output image size is determined by input size 
        [Uo,Vo] = imeshgrid(im);

    else
        % else from specified size
        [Uo,Vo] = meshgrid(1:opt.outsize(1), 1:opt.outsize(2));
    end



    % create the coordinate matrices for warping
    [Ui,Vi] = imeshgrid(im);


    % rotation and scale
    R = rotz(angle);
    uc = nc/2; vc = nr/2;
    Uo2 = 1/opt.scale*(R(1,1)*(Uo-uc)+R(2,1)*(Vo-vc))+uc;
    Vo2 = 1/opt.scale*(R(1,2)*(Uo-uc)+R(2,2)*(Vo-vc))+vc;

    Uo = Uo2;
    Vo = Vo2;

    
    if opt.crop
        trimx = abs(nr/2*sin(angle));
        trimy = abs(nc/2*sin(angle));
        if opt.scale < 1
            trimx = trimx + nc/2*(1-opt.scale);
            trimy = trimy + nr/2*(1-opt.scale);
        end
        trimx = ceil(trimx) + 1;
        trimy = ceil(trimy) + 1;
        trimx
        trimy
        Uo = Uo(trimy:end-trimy,trimx:end-trimx);
        Vo = Vo(trimy:end-trimy,trimx:end-trimx);

    end

    for k=1:size(im,3)
        im2(:,:,k) = interp2(Ui, Vi, im(:,:,k), Uo, Vo, 'linear', opt.extrapval);
    end

    if is_int
        im2 = iint(im2);
    end
