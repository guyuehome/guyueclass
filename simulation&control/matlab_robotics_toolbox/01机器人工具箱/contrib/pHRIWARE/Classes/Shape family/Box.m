%BOX Collision checking class for box-type primitive shapes
%
% Subclass of the Shape class. The x, y and z axes of the transform
% describes the direction of the box's edges, and the translation
% component describes the centre of the box. The scale vector defines
% the edges' half-lengths.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) box = Box(T, s)
%  (2) box = Box(T, s, ...)
%  (3) box = Box()
%
%  (2) is as per (1) but with other properties set in name-value pairs
%  (3) returns a box with default values
%
% Outputs:
%  box : Box object
%
% Inputs:
%  T   : Transformation matrix. If empty, defaults to eye(4).
%  s   : Scale vector. If empty, defaults to [1 1 1].
%  ... : Options - other properties in name-value pairs
%
% See documentation for information on properties and methods
% (type doc Shape into the command window)
%
% See also Box.faces Box.n Cone Curvilinear Cylinder.Cylinder 
% Ellipsoid.Ellipsoid Shape Sphere.Sphere

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.

classdef Box < Shape
    
    properties (Dependent)
        volume
    end
    
    properties
        %FACES  Logical vector of which faces to plot
        % For Boxes, faces is a 1x6 vector, whose elements correspond
        % to the faces in the negative y-z, x-z, x-y and positive y-z,
        % x-z, x-y planes, with respect to that shape's frame, in that
        % order
        faces = true(1,6)
        
        %N "Resolution" of the shape, for plotting
        % For Boxes, each face will comprise of an nxn grid of points
        n = 2
    end
    
    methods
        function box = Box(T, s, varargin)
            if ~nargin
                T = [];
                s = [];
            end
            
            box = box@Shape(T, s, varargin{:});
        end
        
        function vargout = plot(box, varargin)
            [box, frames] = parseopts(box,varargin{:});
            
            [X, Y, Z] = box.cloud;
            
            h = zeros(1,6);
            for f = 1: 6
                if box.faces(f)
                    h(f) = surface(X(:,:,f), Y(:,:,f), Z(:,:,f));
                end
            end     
            h = h(box.faces);

            set(h, ...
                'FaceColor', box.FaceColor, ...
                'FaceAlpha', box.FaceAlpha, ...
                'EdgeColor', box.EdgeColor, ...
                'EdgeAlpha', box.EdgeAlpha);
            
            if ~isempty(frames), h = ellipsoid.animate(h, frames); end         
            if nargout, vargout = h; end
        end
        
        function [X, Y, Z] = cloud(box)
            ptsAlongEdge = linspace(-1,1,box.n);
            [y, z] = meshgrid(ptsAlongEdge,ptsAlongEdge);
            x = -ones(size(z));
            xa = cat(3, x  , y  , z  , x+2, y  , z  );
            ya = cat(3, y  , x  , y  , y  , x+2, y  );
            za = cat(3, z  , z  , x  , z  , z  , x+2);
            
            [X, Y, Z] = box.sat(xa, ya, za);
        end
        
        function inside = check(box, varargin)
            switch length(varargin)
                case 0
                    if box.sym == 8
                        syms x y z real
                        points = [x, y, z, 1];
                    else
                        inside = @box.check;
                        return;
                    end
                case 1
                    points = [varargin{1},ones(size(varargin{1},1),1)];
                case 3
                    x = varargin{1}(:);
                    y = varargin{2}(:);
                    z = varargin{3}(:);
                    points = [x, y, z, ones(size(x))];
            end

            tpts = box.transform \ points';
            tpts = abs(tpts');
            
            inX = tpts(:,1) < box.scale(1);
            
            inY = tpts(:,2) < box.scale(2);
            
            inZ = tpts(:,3) < box.scale(3);
            
            inside = inX & inY & inZ;   
            
            if isa(inside,'sym'), inside = sym2func(inside); end
        end
        
        function v = get.volume(box)
            v = 8*prod(box.scale);
        end

        function box = set.faces(box, f)
            box.faces = f & [1 1 1 1 1 1];
        end
        
    end
    
end

