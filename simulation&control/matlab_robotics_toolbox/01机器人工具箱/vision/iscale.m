%ISCALE Scale an image
%
% OUT = ISCALE(IM, S) is a version of IM scaled in both directions by S
% which is a real scalar.  S>1 makes the image larger, S<1 makes it smaller.
%
% Options::
% 'outsize',S     set size of OUT to HxW where S=[W,H]
% 'smooth',S      initially smooth image with Gaussian of standard deviation
%                 S (default 1).  S=[] for no smoothing.
%
% See also IREPLICATE, IDECIMATE, IROTATE.


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

function im2 = iscale(im, factor, varargin)

    outsize = [];
    bgcolor = 0;
    
    opt.outsize = [];
    opt.smooth = 1;

    opt = tb_optparse(opt, varargin);
    
    if isfloat(im)
        is_int = false;
    else
        is_int = true;
        im = idouble(im);
    end

    if ~isempty(opt.smooth)
        im = ismooth(im, opt.smooth);    % smooth the image to prevent aliasing % TODO should depend on scale factor
    end

    [nr,nc,np] = size(im);

    if isempty(opt.outsize)
        nrs = floor(nr*factor);
        ncs = floor(nc*factor);
    else
        % else from specified size
        % output image size is determined by input size and scale factor
        ncs = outsize(1);
        nrs = outsize(2);
    end

    % create the coordinate matrices for warping
    [U,V] = imeshgrid(im);
    [Uo,Vo] = imeshgrid([ncs, nrs]);

    % the inverse function
    Uo = Uo/factor;
    Vo = Vo/factor;

    for k=1:size(im,3)
        im2(:,:,k) = interp2(U, V, im(:,:,k), Uo, Vo, 'linear');
    end

    if is_int
        im2 = iint(im2);
    end
