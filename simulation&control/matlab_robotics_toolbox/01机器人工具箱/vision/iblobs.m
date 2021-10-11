%IBLOBS	Blob features
%
% F = IBLOBS(IM, OPTIONS) is a vector of RegionFeature objects that
% describe each connected region in the image IM.
%
% Options::
%  'aspect',A        set pixel aspect ratio, default 1.0
%  'connect',C	     set connectivity, 4 (default) or 8
%  'greyscale'	     compute greyscale moments 0 (default) or 1
%  'boundary'        compute boundary (default off)
%  'area',[A1,A2]    accept only blobs with area in the interval A1 to A2
%  'shape',[S1,S2]   accept only blobs with shape in the interval S1 to S2
%  'touch',T         accept only blobs that touch (1) or do not touch (0)
%                    the edge (default accept all)
%  'class',C         accept only blobs of pixel value C (default all)
%
% The RegionFeature object has many properties including:
%
%  uc            centroid, horizontal coordinate
%  vc            centroid, vertical coordinate
%  p             centroid (uc, vc)
%  umin          bounding box, minimum horizontal coordinate
%  umax          bounding box, maximum horizontal coordinate
%  vmin          bounding box, minimum vertical coordinate
%  vmax          bounding box, maximum vertical coordinate
%  area          the number of pixels
%  class         the value of the pixels forming this region
%  label         the label assigned to this region
%  children      a list of indices of features that are children of this feature
%  edgepoint     coordinate of a point on the perimeter
%  edge          a list of edge points 2xN matrix
%  perimeter     edge length (pixels)
%  touch         true if region touches edge of the image
%  a             major axis length of equivalent ellipse
%  b             minor axis length of equivalent ellipse
%  theta         angle of major ellipse axis to horizontal axis
%  shape         aspect ratio b/a (always <= 1.0)
%  circularity   1 for a circle, less for other shapes
%  moments       a structure containing moments of order 0 to 2
%
% References::
%  - Robotics, Vision & Control, Section 13.1,
%    P. Corke, Springer 2011.
%  - METHODS TO ESTIMATE AREAS AND PERIMETERS OF BLOB-LIKE OBJECTS: A COMPARISON
%    Luren Yang, Fritz Albregtsen, Tor Lgnnestad and Per Grgttum
%    IAPR Workshop on Machine Vision Applications Dec. 13-15, 1994, Kawasaki
%  - Area and perimeter measurement of blobs in discrete binary pictures. 
%    Z.Kulpa.
%    Comput. Graph. Image Process., 6:434-451, 1977.
%
% Notes::
% - The RegionFeature objects are ordered by the raster order of the top most
%   point (smallest v coordinate) in each blob.
% - Circularity is computed using the raw perimeter length scaled down by Kulpa's
%   correction factor.
%
% See also RegionFeature, ILABEL, IDISPLABEL, IMOMENTS.


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

function [features,labimg] = iblobs(im, varargin)
	
	[nr,nc] = size(im);

    opt.area = [0 Inf];
    opt.shape = [0 Inf];
    opt.class = NaN;
    opt.touch = NaN;
    opt.aspect = 1;
    opt.connect = 4;
    opt.greyscale = false;
    opt.moments = false;
    opt.boundary = false;

    opt = tb_optparse(opt, varargin);

    % HACK ilabel should take int image
	[li,nl,parent,color,edge] = ilabel(im, opt.connect);

	blob = 0;
	for i=1:nl,
		binimage = (li == i);

		% determine the blob extent
		[y,x] = find(binimage);
		umin = min(x); umax = max(x);
		vmin = min(y); vmax = max(y);

        % it touches the edge if its parent is 0
		t = (parent(i) == 0);

        % compute the moments
		if opt.greyscale
			F = imoments(binimage .* im, 'aspect', opt.aspect);
		else
			F = imoments(binimage, 'aspect', opt.aspect);
		end

        % compute shape property, accounting for degenerate case
		if F.a == 0,
			shape = NaN;
		else
			shape = F.b / F.a;
		end

		% apply various filters
		if 	((t == opt.touch) || isnan(opt.touch)) && ...
            ((color(i) == opt.class) || isnan(opt.class)) && ...
			(F.area_ >= opt.area(1)) && ...
			(F.area_ <= opt.area(2)) && ...
			(					...
				isnan(shape) ||			...
				(               ...
					(shape >= opt.shape(1)) &&	...
					(shape <= opt.shape(2))	...
				)				...
			)

            % this blob matches the filter

            % record a perimeter point
            [y,x] = ind2sub(size(im), edge(i));
            F.edgepoint = [x y];    % a point on the perimeter

            % optionally follow the boundary
            if opt.boundary
                e = edgelist(im, [x y]);
                F.edge = e';

                e = diff([e; e(1,:)])';
                
                % compute length:
                %   - 1 for horizontal/vertical segment
                %   - sqrt(2) for diagnonal
                %
                % Apply Kulpa's correction factor when computing
                % circularity
                kulpa = pi/8*(1+sqrt(2));
                F.perimeter_ = sum( colnorm(e) );
                
                F.circularity_ = 4*pi*F.area_/ (F.perimeter_*kulpa)^2;
            end

            % set object properties
			F.umin = umin;
			F.umax = umax;
			F.vmin = vmin;
			F.vmax = vmax;
			F.touch_ = t;
            F.parent = parent(i);

			F.shape_ = shape;
            F.label_ = i;
            F.class_ = color(i);

            % save it in the feature vector
			blob = blob+1;
			features(blob) = F;
		end
	end

    if blob == 0
        features = [];
    end

    % add children property
    % the numbers in the children property refer to elements in the feature vector
    for i=1:length(features)
        parent = features(i).parent;
        for F=features
            if F.label == parent
                F.children = [F.children i];
            end
        end
    end

	%fprintf('%d blobs in image, %d after filtering\n', nl, blob);
    if nargout > 1
        labimg = li;
    end
