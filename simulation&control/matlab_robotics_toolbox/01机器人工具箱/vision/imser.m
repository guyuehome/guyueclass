%IMSER Maximally stable extremal regions
%
% LABEL = IMSER(IM, OPTIONS) is a segmentation of the greyscale image IM (HxW)
% based on maximally stable extremal regions.  LABEL (HxW) is an image where 
% each element is the integer label assigned to the corresponding pixel in IM.
% The labels are consecutive integers starting at zero.
%
% [LABEL,NREG] = IMSER(IM, OPTIONS) as above but NREG is the number of regions
% found, or one plus the maximum value of LABEL.
%
% Options::
% 'dark'    looking for dark features against a light background (default)
% 'light'   looking for light features against a dark background
%
% Example::
%
%     im = iread('castle_sign2.png', 'grey', 'double');
%     [label,n] = imser(im, 'light');
%     idisp(label)
%
% Notes::
% - Is a wrapper for vl_mser, part of VLFeat (vlfeat.org), by Andrea Vedaldi
%   and Brian Fulkerson.
% - vl_mser is a MEX file.
%
% Reference::
%
% "Robust wide-baseline stereo from maximally stable extremal regions",
% J. Matas, O. Chum, M. Urban, and T. Pajdla, 
% Image and Vision Computing,
% vol. 22, pp. 761-767, Sept. 2004.
%
% See also ITHRESH, IGRAPHSEG.


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

function [all,nsets] = imser(im, varargin)
    if size(im,3) > 1
        error('monochrome images only');
    end

    % process the argument list.
    %  we add two arguments 'light', 'dark' for the wrapper, the rest get
    % get passed to MSER.
    opt.invert = {'dark', 'light'};

    [opt,mser_args] = tb_optparse(opt, varargin);

    % add default args if none given
    if isempty(mser_args)
        mser_args = {'BrightOnDark', 0, 'DarkOnBright', 1, 'MinArea', 0.0001, 'MaxArea', 0.1};
    end

    if opt.verbose
        mser_args = [mser_args 'Verbose' ];
    end

    invert = true;
    switch opt.invert
        case 'dark'
            invert = false;
        case 'light'
            invert = true;
    end

    % MSER operates on a uint8 image
    if isfloat(im)
        if invert
            im = 1.0-im;
        end
        im = iint(im);
    else
        if invert
            im = 255-im;
        end
    end

    [R,F] = vl_mser(im, mser_args{:});
    fprintf('%d MSERs found\n', length(R)+1);

    %f1
    %idisp(im);

    all = zeros( size(im));
    count = 1;
    for r=abs(R)'
        bim = im <= im(r);
        % HACK bim = im <= im(r);
        lim = ilabel(bim);
        mser_blob = lim == lim(r);

        %sum(mser_blob(:))

        %idisp(mser_blob)
        all(mser_blob) =  count;
        count = count + 1;
        [row,col] = ind2sub(size(bim), r);
        %hold on
        %plot(col, row, 'g*');
        %pause(.2)
    end
    nsets = count;
