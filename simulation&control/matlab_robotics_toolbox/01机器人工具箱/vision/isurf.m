%ISURF SURF feature extractor
%
% SF = ISURF(IM, OPTIONS) returns a vector of SurfPointFeature objects
% representing scale and rotationally invariant interest points in the
% image IM.
%
% The SurfPointFeature object has many properties including:
%  u            horizontal coordinate
%  v            vertical coordinate
%  strength     feature strength
%  descriptor   feature descriptor (64x1 or 128x1)
%  sigma        feature scale
%  theta        feature orientation [rad]
%
% Options::
% 'nfeat',N      set the number of features to return (default Inf)
% 'thresh',T     set Hessian threshold.  Increasing the threshold reduces
%                the number of features computed and reduces computation time.
% 'octaves',N    number of octaves to process (default 5)
% 'extended'     return 128-element descriptor (default 64)
% 'upright'      don't compute rotation invariance
% 'suppress',R   set the suppression radius (default 0).  Features are not
%                returned if they are within R [pixels] of an earlier (stronger)
%                feature.
% Example::
%
% Load the image
%         im = iread('lena.pgm');
% Find the 10 strongest SURF features
%         sf = isurf(im, 'nfeat', 10);
% and overlay them on the original image as blue circles
%         idisp(im);
%         sf.plot_scale()
%
% Notes::
% - Color images, or sequences, are first converted to greyscale.
% - Features are returned in descending strength order
% - If IM is HxWxN it is considered to be an image sequence and F is a cell 
%   array with N elements, each of which is the feature vectors for the 
%   corresponding image in the sequence.
% - Wraps an M-file implementation of OpenSurf by D. Kroon (U. Twente) or
%   a MEX-file OpenCV wrapper by Petter Strandmark.
% - The sign of the Laplacian is not retained.
% - The SURF algorithm is covered by an extensive suite of international
%   patents including US 8,165,401, EP 1850270 held by Toyota, KU Leuven
%   and ETHZ.  See http://www.kooaba.com/en/plans_and_pricing/ip_licensing
% 
% Reference::
% "SURF: Speeded Up Robust Features", 
% Herbert Bay, Andreas Ess, Tinne Tuytelaars, Luc Van Gool,
% Computer Vision and Image Understanding (CVIU), 
% Vol. 110, No. 3, pp. 346--359, 2008
%
%
% See also SurfPointFeature, ISIFT, ICORNER.


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

function features = isurf(im, varargin)

    opt.suppress = 0;
    opt.nfeat = Inf;
    opt.extended = true;
    opt.upright = false;
    opt.octaves = 5;
    opt.thresh = [];
    opt.id = [];

    [opt,arglist] = tb_optparse(opt, varargin);

    if iscell(im)
        % images provided as a cell array, return a cell array
        % of SIFT object vectors
        fprintf('extracting SIFT features for %d greyscale images\n', length(im));
        features = {};
        for i=1:length(im)
            sf = isurf(im{i}, 'setopt', opt, 'id', i);

            features{i} = sf;
            fprintf('.');
        end
        fprintf('\n');
        return
    end

    % convert color image to greyscale
    if iscolor(im)
       im = imono(im);
    end

    if ndims(im) > 2

        % TODO sequence of color images..

        % images provided as an array, return a cell array
        % of SURF object vectors
        if opt.verbose
            fprintf('extracting SURF features for %d greyscale images\n', size(im,3));
        end
        features = {};
        for i=1:size(im,3)
            sf = isurf(im(:,:,i), 'setopt', opt);
            for j=1:length(sf)
                sf(j).image_id_ = i;
            end
            features{i} = sf;
            fprintf('.');
        end
        if opt.verbose
            fprintf('\n');
        end
        return
    end


    % do SURF using a static method that wraps the implementation from:
    % OpenSURF for Matlab
    %   written by D.Kroon University of Twente (July 2010)
    %   based on the C++ implementation by Chris Evans

    Ipts = SurfPointFeature.surf(im, opt);

    % Ipts is a structure array with elements x, y, scale, orientation, descriptor


    try
        % sort into descending order of corner strength
        [z,k] = sort([Ipts.strength], 'descend');
        Ipts = Ipts(:,k);
    catch
        % will fail if no features found
        features = [];
        return;
    end
    
    
    fprintf('%d corners found (%.1f%%), ', length(Ipts), ...
        length(Ipts)/prod(size(im))*100);

    % allocate storage for the objects
    n = min(opt.nfeat, length(Ipts));

    features = [];
    i = 1;
    while i<=n
        if i > length(Ipts)
            break;
        end

        % enforce separation between corners
        % TODO: strategy of Brown etal. only keep if 10% greater than all within radius
        if (opt.suppress > 0) && (i>1)
            d = sqrt( ([features.v]'-Ipts(i).y).^2 + ([features.u]'-Ipts(i).x).^2 );
            if min(d) < opt.suppress
                continue;
            end
        end
        f = SurfPointFeature(Ipts(i).x, Ipts(i).y, Ipts(i).strength);
        f.scale_ = Ipts(i).scale;
        f.theta_ = Ipts(i).orientation;
        f.descriptor_ = cast(Ipts(i).descriptor, 'single');
        f.image_id_ = opt.id;

        features = [features f];
        i = i+1;
    end
    fprintf(' %d corner features saved\n', i-1);
