%COLLISIONMODEL Class to conduct point-primitive collision checking
%
% Group objects from the Shape heirachy and/or other CollisionModel
% objects for collision checking functionality. Once made,
% CollisionModel objects cannot be modified. Primitives are ordered in
% decreasing volume to help optimise the collision checking process.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) cmdl = CollisionModel(prim1, prim2, ..., primN)
%  (2) cmdl = CollisionModel(note, prim1, prim2, ..., primN)
%
%  (2) is as per (1) but also includes a text note about the object
%
% Outputs:
%  cmdl : CollisionModel object
%
% Inputs:
%  prim : Primitive shape(s). May either be scalar, array, or cell of
%          objects which are from the Shape class heirachy, or a scalar
%          CollisionModel object.
%  note : Short text description of object for user reference
%
% See documentation for information on properties and methods
% (type doc CollisionModel into the command window)
%
% See also Box.Box collisions Cone Curvilinear Cylinder.Cylinder
%  Ellipsoid.Ellipsoid Shape Sphere.Sphere

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
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.

classdef CollisionModel
    
    properties (SetAccess = private)
        primitives % Primitive shapes of the CollisionModel
        checkFuns % Function handles to check each primitive
        % Short text description of object for user reference
        note = '<Add a description with note property>'
    end
    
    methods
        function cmdl = CollisionModel(varargin)
            if ischar(varargin{1})
                cmdl.note = varargin{1};
                shapes = varargin(2:end);
            else
                shapes = varargin;       
            end
            
            i = 1;
            while i <= length(shapes)
                if isa(shapes{i},'CollisionModel')
                    cell = shapes{i}.primitives;
                elseif ~isscalar(shapes{i})
                    cell = num2cell(shapes{i});
                elseif iscell(shapes{i})
                    cell = shapes{i};
                else
                    cell = [];
                end
                if ~isempty(cell)
                    shapes = [shapes(1:i-1) cell(:)' shapes(i+1:end)];
                end
                i = i + 1;
            end
            
            for i = length(shapes): -1: 1
                negVol(i) = -shapes{i}.volume;
                funcs{i} = shapes{i}.check;
            end
            
            [~, order] = sort(negVol);
            cmdl.checkFuns = funcs(order);
            cmdl.primitives = shapes(order);
        end
        
        function h = plot(cmdl, varargin)
            %PLOT Plot the CollisionModel
            %
            % As per the Shape method, but options affect all primitives
            %
            % See also Shape.plot
            
            n = length(cmdl.primitives);
            hold on
            if nargout == 1, h = []; end
            for i = 1: n
                hn = cmdl.primitives{i}.plot(varargin{:});
                if nargout == 1, h = [h, hn]; end
            end
            hold off
        end
        
        function c = collision(cmdl, pts)
            %COLLISION Determine if a collision is occuring
            %
            % As per the check method, but c is a scalar value, and
            % will return as soon as a collision is determined
            %
            % See also CollisionModel.check
            
            n = length(cmdl.primitives);
            c = false;
            for i = 1: n
                if any(cmdl.checkFuns{i}(pts(:,1),pts(:,2),pts(:,3)));
                    c = true;
                    return;
                end
            end
        end
        
        function inside = check(cmdl, pts)
            %CHECK Check if point(s) are inside the CollisionModel
            %
            % As per the Shape method, but c is mxp, where p is the
            % number of primitives
            %
            % See also CollisionModel.collisions Shape.check
            
            n = length(cmdl.primitives);
            inside = zeros(size(pts,1),n);
            for i = 1: n
                inside(:,i) = ...
                    cmdl.checkFuns{i}(pts(:,1), pts(:,2), pts(:,3));
            end
        end
    end
    
end

