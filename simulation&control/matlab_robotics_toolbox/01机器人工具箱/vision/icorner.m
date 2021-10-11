%ICORNER Corner detector
%
% F = ICORNER(IM, OPTIONS) is a vector of PointFeature objects describing
% corner features detected in the image IM.  This is a non-scale space detector 
% and by default the Harris method is used but Shi-Tomasi and Noble are also
% supported.
%
% If IM is an image sequence a cell array of PointFeature vectors for the 
% correspnding frames of IM.
%
% The PointFeature object has many properties including:
%  u            horizontal coordinate
%  v            vertical coordinate
%  strength     corner strength
%  descriptor   corner descriptor (vector)
%
% See PointFeature for full details
%
% Options::
% 'detector',D      choose the detector where D is one of 'harris' (default),
%                   'noble' or 'klt'
% 'sigma',S         kernel width for smoothing (default 2)
% 'deriv',D         kernel for gradient (default kdgauss(2))
% 'cmin',CM         minimum corner strength
% 'cminthresh',CT   minimum corner strength as a fraction of maximum corner 
%                   strength
% 'edgegap',E       don't return features closer than E pixels to the edge of 
%                   image (default 2)
% 'suppress',R      don't return a feature closer than R pixels to an earlier 
%                   feature (default 0)
% 'nfeat',N         return the N strongest corners (default Inf)
% 'k',K             set the value of k for the Harris detector
% 'patch',P         use a PxP patch of surrounding pixel values as the 
%                   feature vector.  The vector has zero mean and unit norm.
% 'color'           specify that IM is a color image not a sequence
%
% Example::
%
% Compute the 100 strongest Harris features for the image
%         c = icorner(im, 'nfeat', 100);
% and overlay them on the image
%         idisp(im);
%         c.plot();
%
% Notes::
% - Corners are processed in order from strongest to weakest.
% - The function stops when:
%     - the corner strength drops below cmin, or
%     - the corner strength drops below cMinThresh x strongest corner, or
%     - the list of corners is exhausted
% - Features are returned in descending strength order
% - If IM has more than 2 dimensions it is either a color image or a sequence
% - If IM is NxMxP it is taken as an image sequence and F is a cell array whose
%   elements are feature vectors for the corresponding image in the sequence.
% - If IM is NxMx3 it is taken as a sequence unless the option 'color' is given
% - If IM is NxMx3xP it is taken as a sequence of color images and F is a cell
%   array whose elements are feature vectors for the corresponding color image 
%   in the sequence.
% - The default descriptor is a vector [Ix* Iy* Ixy*] which are the unique
%   elements of the structure tensor, where * denotes squared and smoothed.
% - The descriptor is a vector of float types to save space
%
% References::
% - "A combined corner and edge detector", 
%   C.G. Harris and M.J. Stephens,
%   Proc. Fourth Alvey Vision Conf., Manchester, pp 147-151, 1988.
% - "Finding corners", 
%   J.Noble, 
%   Image and Vision Computing, vol.6, pp.121-128, May 1988.
% - "Good features to track",
%   J. Shi and C. Tomasi, 
%   Proc. Computer Vision and Pattern Recognition, pp. 593-593,
%   IEEE Computer Society, 1994.
%  - Robotics, Vision & Control, Section 13.3,
%    P. Corke, Springer 2011.
%
% See also PointFeature, ISURF.



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


function [features, corner_strength] = icorner(im, varargin)

    % TODO, can handle image sequence, return 3D array of corner_strength if requested 
    % and cell array of corner vectors
    % handle tiling

    % parse options into parameter struct
    opt.k = 0.04;
    opt.deriv = kdgauss(2);

    opt.cmin = 0;
    opt.cminthresh = 0.0;
    opt.edgegap = 2;
    opt.nfeat = Inf;
    opt.sigma = 2;
    opt.patch = 0;
    opt.detector = {'harris', 'noble', 'st'};
    opt.color = false;

    opt.suppress = 0;
    opt.nfeat = 100;

    [opt,arglist] = tb_optparse(opt, varargin);

    if opt.patch > 0
        opt.edgegap = opt.patch;
    end

    if iscell(im)
        % images provided as a cell array, return a cell array
        % of corner object vectors
        if opt.verbose
            fprintf('extracting corner features for %d images\n', length(im));
        end
        features = {};
        for i=1:length(im)
            f = icorner(im{i}, 'setopt', opt);
            for j=1:length(f)
                f(j).image_id = i;
            end
            features{i} = f;
            fprintf('.');
        end
        if opt.verbose
            fprintf('\n');
        end
        return
    end


    if ndims(im) > 2

        % images provided as an array, return a cell array
        % of corner object vectors

        if ndims(im) == 4 && size(im,3) == 3 && opt.color
            fprintf('extracting corner features for color %d images\n', size(im,4));
            features = {};
            % sequence of color images
            for i=1:size(im,k)
                f = icorner(im(:,:,:,i), 'setopt', opt);
                for j=1:length(f)
                    %f(j).image_id_ = i;
                end
                features{i} = f;
                fprintf('.');
            end
            fprintf('\n');
            return
        elseif ndims(im) == 3 && ~opt.color
            fprintf('extracting corner features for grey %d images\n', size(im,3));
            features = {};
            % sequence of grey images
            for i=1:size(im,3)
                f = icorner(im(:,:,i), 'setopt', opt);
                for j=1:length(f)
                    %f(j).image_id_ = i;
                end
                features{i} = f;
                fprintf('.');
            end
            fprintf('\n');
            return
        end
    end

    if ndims(im) == 3 & opt.color
        R = double(im(:,:,1));
        G = double(im(:,:,2));
        B = double(im(:,:,3));
        Rx = conv2(R, opt.deriv, 'same');
        Ry = conv2(R, opt.deriv', 'same');
        Gx = conv2(G, opt.deriv, 'same');
        Gy = conv2(G, opt.deriv', 'same');
        Bx = conv2(B, opt.deriv, 'same');
        By = conv2(B, opt.deriv', 'same');

        Ix = Rx.^2+Gx.^2+Bx.^2;
        Iy = Ry.^2+Gy.^2+By.^2;
        Ixy = Rx.*Ry+Gx.*Gy+Bx.*By;
    else
        % compute horizontal and vertical gradients
        im = double(im);
        ix = conv2(im, opt.deriv, 'same');
        iy = conv2(im, opt.deriv', 'same');
        Ix = ix.*ix;
        Iy = iy.*iy;
        Ixy = ix.*iy;
    end

    % smooth them
    if opt.sigma > 0
        Ix = ismooth(Ix, opt.sigma);
        Iy = ismooth(Iy, opt.sigma);
        Ixy = ismooth(Ixy, opt.sigma);
    end

    [nr,nc] = size(Ix);
    npix = nr*nc;

    % computer cornerness
    switch opt.detector
    case 'harris'
        cornerness = (Ix .* Iy - Ixy.^2) - opt.k * (Ix + Iy).^2;
    case 'noble'
        cornerness = (Ix .* Iy - Ixy.^2) ./ (Ix + Iy);
    case 'st'
        cornerness = zeros(size(Ix));
        for i=1:npix
            lambda = eig([Ix(i) Ixy(i); Ixy(i) Iy(i)]);
            cornerness(i) = min(lambda);
        end
    end

    % compute maximum value around each pixel
    cmax = imorph(cornerness, [1 1 1;1 0 1;1 1 1], 'max');

    % if pixel exceeds this, its a local maxima, find index
    cindex = find(cornerness > cmax);

    fprintf('%d corners found (%.1f%%), ', length(cindex), ...
        length(cindex)/npix*100);

    % remove those near edges
    [y, x] = ind2sub(size(cornerness), cindex);
    e = opt.edgegap;
    k = (x>e) & (y>e) & (x < (nc-e)) & (y < (nr-e));
    cindex = cindex(k);

    % corner strength must exceed an absolute minimum
    k = cornerness(cindex) < opt.cmin;
    cindex(k) = [];

    % sort into descending order
    cval = cornerness(cindex);		    % extract corner values
    [z,k] = sort(cval, 'descend');	% sort into descending order
    cindex = cindex(k)';
    cmax = cornerness( cindex(1) );   % take the strongest feature value

    % corner strength must exceed a fraction of the maximum value
    k = cornerness(cindex)/cmax < opt.cminthresh;
    cindex(k) = [];

    % allocate storage for the objects
    n = min(opt.nfeat, numcols(cindex));

    features = [];
    i = 1;
    while i <= n
        if i > length(cindex)
            break;
        end
        K = cindex(i);
        c = cornerness(K);

        % get the coordinate
        [y, x] = ind2sub(size(cornerness), K);

        % enforce separation between corners
        % TODO: strategy of Brown etal. only keep if 10% greater than all within radius
        if (opt.suppress > 0) && (i>1)
            d = sqrt( sum((features.v'-y).^2 + (features.u'-x).^2) );
            if min(d) < opt.suppress
                continue;
            end
        end

        % ok, this one is for keeping
        f = PointFeature(x, y, c);
        if opt.patch == 0
            f.descriptor_ = cast([Ix(K) Iy(K) Ixy(K)]', 'single');
        else
            % if opt.patch is finite, then return a vector which is the local image
            % region as a vector, zero mean, and normalized by the norm.
            % the dot product of this with another descriptor is the ZNCC similarity measure
            w2 = opt.patch;
            d = im(y-w2:y+w2,x-w2:x+w2);
            d = d(:);
            d = d - mean(d);
            f.descriptor_ = cast( d / norm(d), 'single');
        end

        features = [features f];
        i = i+1;
    end
    fprintf(' %d corner features saved\n', i-1);

    % sort into descending order of strength
    [z,k] = sort(-features.strength);
    features = features(k);

    if nargout > 1
        corner_strength = cornerness;
    end
