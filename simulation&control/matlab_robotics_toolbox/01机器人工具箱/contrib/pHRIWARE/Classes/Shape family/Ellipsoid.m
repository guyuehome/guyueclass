%ELLIPSOID Collision checking class for ellipsoid-type primitive shapes
%
% Subclass of the Shape class. The x, y and z axes of the transform
% describes the direction of the ellipsoid's radii, and the translation
% component describes the centre of the ellipsoid. The scale vector 
% defines the size of the radii.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) ellipsoid = Ellipsoid(T, s)
%  (2) ellipsoid = Ellipsoid(T, s, ...)
%  (3) ellipsoid = Ellipsoid()
%
%  (2) is as per (1) but with other properties set in name-value pairs
%  (3) returns an ellipsoid with default values
%
% Outputs:
%  ellipsoid : Ellipsoid object
%
% Inputs:
%  T   : Transformation matrix. If empty, defaults to eye(4).
%  s   : Scale vector. If empty, defaults to [1 1 1].
%  ... : Options - other properties in name-value pairs
%
% See documentation for information on properties and methods
% (type doc Shape into the command window)
%
% See also Box.Box Cone Curvilinear Cylinder.Cylinder Ellipsoid.faces
% Ellipsoid.n Shape Sphere.Sphere

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

classdef Ellipsoid < Shape
    
    properties
        %FACES  Logical vector of which faces to plot
        % For Ellipsoids, faces is not currently used
        faces
        
        %N "Resolution" of the shape, for plotting
        % For Ellipsoids, it is the equivalent of sphere(n)
        n = 20
    end
    
    properties (Dependent)
        volume
    end
    
    methods
        function ellipsoid = Ellipsoid(T, s, varargin)
            if ~nargin
                T = [];
                s = [1 1.5 2];
            end
            
            ellipsoid = ellipsoid@Shape(T, s, varargin{:}); 
        end
           
        function vargout = plot(ellipsoid, varargin)
            [ellipsoid, frames] = ellipsoid.parseopts(varargin{:});
            
            [X, Y, Z] = ellipsoid.cloud;
            
            h = surface(X,Y,Z);
            
            set(h, ...
                'FaceColor', ellipsoid.FaceColor, ...
                'FaceAlpha', ellipsoid.FaceAlpha, ...
                'EdgeColor', ellipsoid.EdgeColor, ...
                'EdgeAlpha', ellipsoid.EdgeAlpha);
            
            if ~isempty(frames), h = ellipsoid.animate(h, frames); end
            if nargout, vargout = h; end
        end
        
        function [X, Y, Z] = cloud(ellipsoid)
            [x, y, z] = sphere(ellipsoid.n);
            [X, Y, Z] = ellipsoid.sat(x, y, z);
        end
             
        function inside = check(ellipsoid, varargin)
            switch length(varargin)
                case 0
                    if ellipsoid.sym == 8
                        syms x y z real
                        points = [x, y, z, 1];
                    else
                        inside = @ellipsoid.check;
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
            
            tpts = ellipsoid.transform \ points';
            tpts = tpts';
            
            r1 = ellipsoid.scale(1);
            
            r2 = ellipsoid.scale(2);
            
            r3 = ellipsoid.scale(3);
            
            inside = (tpts(:,1).^2/r1^2) + (tpts(:,2).^2/r2^2) + ...
                (tpts(:,3).^2/r3^2) < 1;
            
            if isa(inside,'sym'), inside = sym2func(inside); end
        end
        
        function v = get.volume(ellipsoid)
            v = 4*pi*prod(ellipsoid.scale)/3;
        end
    end
    
end

