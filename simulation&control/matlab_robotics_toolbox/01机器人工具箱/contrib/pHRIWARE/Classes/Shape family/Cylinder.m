%CYLINDER Collision checking class for cylinder-type primitive shapes
%
% Subclass of the Curvilinear class. The radius property is set to 1 -
% Use scale property to control cylinder radii lengths.May be circular
% or elliptical cylinders.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) cylinder = Cylinder(T, s)
%  (2) cylinder = Cylinder(T, s, ...)
%  (3) cylinder = Cylinder(P, r, ...)
%  (3) cylinder = Cylinder()
%
%  (2) is as per (1) but with other properties set in name-value pairs
%  (3) is as per (1) or (2) but is defined by two points and a radius -
%       the cross-section thus must be circular and not elliptical
%  (4) returns a cylinder with default values
%
% Outputs:
%  cylinder : Cylinder object
%
% Inputs:
%  T      : Transformation matrix. If empty, defaults to eye(4).
%  s      : Scale vector. If empty, defaults to [1 1 1].
%  P      : Cell, whose first element is the origin, and the second
%            element is the other endpoint. Cell and points may be
%            either column or row vectors.
%  r      : Scalar radius value, for syntax mode (3)
%  ...    : Options - other properties in name-value pairs
%
% See documentation for information on properties and methods
% (type doc Shape into the command window)
%
% See also Box.Box Cone Curvilinear Ellipsoid.Ellipsoid Shape
% Sphere.Sphere

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

classdef Cylinder < Curvilinear

    methods
        function cylinder = Cylinder(T, s, varargin)
            if ~nargin
               T = [];
               s = [];
            end
            
            cylDefaults = [{'n',2}, varargin];
            cylinder = cylinder@Curvilinear(T, s, 1, cylDefaults{:});
        end
    end
    
end

