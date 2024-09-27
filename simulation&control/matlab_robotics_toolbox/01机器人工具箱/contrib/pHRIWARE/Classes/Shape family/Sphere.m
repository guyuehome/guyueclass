%SPHERE Collision checking class for sphere-type primitive shapes
%
% Subclass of the Ellipsoid class. The orientation needs not be given,
% but may be if desired. A radius is given instead of the scale vector.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) sphere = Sphere(T, r)
%  (2) sphere = Sphere(T, r, ...)
%  (3) sphere = Sphere(p, r, ...)
%  (4) sphere = Sphere()
%
%  (2) is as per (1) but with other properties set in name-value pairs
%  (3) as per (1) or (2) but sphere origin given only, not transform.
%       Orientation is denoted by identity matrix.
%  (4) returns a sphere with default values
%
% Outputs:
%  sphere : Sphere object
%
% Inputs:
%  T   : Transformation matrix. If empty, defaults to eye(4).
%  r   : Radius of sphere. If empty, defaults to [1 1 1].
%  p   : Point of sphere's origin. May be row or column vector.
%  ... : Options - other properties in name-value pairs
%
% See documentation for information on properties and methods
% (type doc Shape into the command window)
%
% See also Box.Box Cone Curvilinear Cylinder.Cylinder
% Ellipsoid.Ellipsoid Shape Sphere.faces Sphere.n

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

classdef Sphere < Ellipsoid
    
    methods
        function sphere = Sphere(T, r, varargin)
            if ~nargin
               T = [];
               r = [];
            end
            
            if numel(T) == 3
                T = [eye(3) T(:); 0 0 0 1];
            end
            
            if numel(r) > 1
                error(pHRIWARE('error', 'inputSize'));
            end

            sphere = sphere@Ellipsoid(T, r, varargin{:});
        end
        
    end
    
end

