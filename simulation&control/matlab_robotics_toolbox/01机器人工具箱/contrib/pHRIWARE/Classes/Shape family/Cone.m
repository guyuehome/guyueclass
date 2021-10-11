%CONE Collision checking class for cone-type primitive shapes
%
% Subclass of the Curvilinear class. The base of the cone intersects
% the transformation frame's origin. May be circular or elliptical.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) cone = Cylinder(T, s)
%  (2) cone = Cone(T, s, ...)
%  (3) cone = Cone(P, r, ...)
%  (3) cone = Cone()
%
%  (2) is as per (1) but with other properties set in name-value pairs
%  (3) is as per (1) or (2) but is defined by two points and a radius -
%       the cross-section thus must be circular and not elliptical
%  (4) returns a cone with default values
%
% Outputs:
%  cone : Cone object
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
% See also Box.Box Curvilinear Cylinder.Cylinder Ellipsoid.Ellipsoid
% Shape Sphere.Sphere

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

classdef Cone < Curvilinear

    methods
        function s = Cone(T, S, varargin)
            if ~nargin
               T = [];
               S = [];
            end
            
            p = @(t)(1-t);
        
            coneDefaults = [{'n',2,'faces',[1 0]}, varargin];
            s = s@Curvilinear(T,S,p,coneDefaults{:});
        end
    end
end
