%SHAPE Abstract superclass for primitive shapes
%
% Defines concrete and abstract properties and methods which are common
% to all shape subclasses (primitives). Also contains some functions
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) shape = Shape(T, s, ...)
%
% Outputs:
%  shape : Shape object. Note as Shape is abstract it cannot be
%           instantiated, hence shape is an object of a Shape subclass.
%
% Inputs:
%  T   : See transform property
%  s   : See scale property (can be input as scalar or 3-vector)
%  ... : Options - other properties in name-value pairs
%
% See documentation for information on properties and methods
% (type doc Shape into the command window)
%
% See also Box.Box CollisionModel Cone Curvilinear Cylinder.Cylinder 
% Ellipsoid.Ellipsoid Sphere.Sphere surface

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

classdef (Abstract) Shape
    
    properties 
        % Short text description of object for user reference
        note = '<Add a description with note property>'
        transform = eye(4) % Transformation matrix of the shape
        scale = [1 1 1] % Scale vector of the shape, [s_x s_y s_z]
        FaceColor = pHRIWARE('blue'); % The surface property
        FaceAlpha = 1 % The surface property
        EdgeColor = pHRIWARE('navy'); % The surface property
        EdgeAlpha = 1 % The surface property
    end
    
    properties (Abstract)
        volume % Volume of shape
        
        %FACES  Logical vector of which faces to plot
        % For Boxes, faces is a 1x6 vector, whose elements correspond
        % to the faces in the negative y-z, x-z, x-y and positive y-z,
        % x-z, x-y planes, with respect to that shape's frame,
        %  in that order
        % For Curvilinears, faces is a 1x2 vector, whose elements
        %  correspond to the end faces whose centres are at the origin
        %  and not at the origin, with respect to that shape's frame,
        %  in that order
        % For Ellipsoids, faces is not currently used
        faces
        
        %N "Resolution" of the shape, for plotting
        % For Boxes, each face will comprise of an nxn grid of points
        % For Curvilinears, it is the equivalent of cylinder(n)
        % For Ellipsoids, it is the equivalent of sphere(n)
        n
    end
    
    properties (Constant)
        sym = exist('sym','class'); % Symbolic Math Toolbox install flag
    end
       
    methods (Abstract)
        %PLOT Plot the shape
        %
        % Plots the shape object to the current figure as a surface
        % object or objects. Only a few options are included, but a
        % graphics handle can be optionally returned for more advanced
        % demands.
        %
        % Copyright (C) Bryan Moutrie, 2013-2014
        % Licensed under the GNU General Public License, see file for
        % statement
        %
        % Syntax:
        %  (1) shape.plot()
        %  (2) shape.plot(...)
        %  (3) h = shape.plot(...)
        %
        %  (1) Plots the shape with the options specified by its
        %       various properties
        %  (2) is as per (1) but can specify over-riding values for
        %       options for the plot only, in name-value pairs
        %  (3) is as per (1) or (2) but returns a handle or vecor of
        %       handles to the surface objects plotted
        %
        % Outputs:
        %  h : graphics handle for each surface plotted
        %
        % Inputs:
        %  ... : Plotting options (which are Shape properties), in
        %         name-value pairs
        %
        % See also Shape.cloud Shape.parseopts  
        h = plot(shape, varargin)
        
        %CLOUD Generate point cloud from shape
        %
        % Generates three arrays corresponding to the x, y and z points
        % of a shape's point data which is readable by the surface
        % function. If the shape is a Box, the third dimension of X, Y,
        % and Z is the points for each face - therefore some points are
        % duplicated (more specifically the corner and edge points)
        %
        % Copyright (C) Bryan Moutrie, 2013-2014
        % Licensed under the GNU General Public License, see file for
        % statement
        %
        % Syntax:
        %  (1) [X, Y, Z] = shape.cloud()
        %
        % Outputs:
        %  X : array of x-coordinates
        %  Y : array of y-coordinates
        %  Z : array of z-coordinates
        %
        % See also Shape.plot surface
        [X, Y, Z] = cloud(shape)  
        
        %CHECK Check if point(s) are inside the shape
        %
        % Check to see if point(s) lie inside a primitive shape. 
        % Points on the surface are treated as outside the shape.
        %
        % Copyright (C) Bryan Moutrie, 2013-2014
        % Licensed under the GNU General Public License, see file for
        % statement
        %
        % Syntax:
        %  (1) c = shape.check(x, y, z)
        %  (2) c = shape.check(p)
        %  (3) f = shape.check()
        %  (4) h = shape.check()
        %
        %  (2) is as per (1) where p = [x, y, z]
        %  (3) Requires the MuPAD symbolic mathematics toolbox
        %  (4) Repalces (3) if no MuPAD symbolic mathematics toolbox
        %
        % Outputs:
        %  c : mx1 vector, where there are m points. Each element is
        %       the logical value of the mth point being inside the
        %       shape
        %  f : anonymous function, f(x, y, z), which reduces check to a
        %       simplified, one-line evaluation
        %  h : function handle, h(x, y, z), OR h(p), which reduces 
        %       check to a simplified, one-line evaluation
        %
        % Inputs:
        %  x : x-coordinate(s) of point(s), may be any size/shape
        %  y : y-coordinate(s) of point(s), may be any size/shape
        %  z : z-coordinate(s) of point(s), may be any size/shape
        %  p : An mx3 matrix, where each column is x, y, z. m may be 1.
        %
        % See also CollisionModel SerialLink.collisions sym2func
        c = check(shape, varargin)   
    end
    
    methods
        function shape = Shape(T, s, varargin)
            shape = shape.parseopts(varargin{:});
            if ~isempty(T), shape.transform = T; end
            if ~isempty(s), shape.scale = s; end
        end
        
        function disp(shape)
            w = whos('shape');
            line1 = [w.class,': ',shape.note];
            line2(1:length(line1)) = '-';
            line3 = 'transform =';
            line4 = ['scale = ',mat2str(shape.scale)];
            line5 = ['volume = ',num2str(shape.volume), ' u^3'];
            if isprop(shape,'radius'),
                if ~isa(shape.radius,'function_handle') %#ok<*MCNPN>
                    line6 = ['radius = ',num2str(shape.radius)];
                else
                    line6 = ['radius = ',func2str(shape.radius)];
                end
            else
                line6 = [];
            end
            line7 = line2;
            line8 = 'Plot options:';
            line9 = ['FaceColor = ',mat2str(shape.FaceColor,2),...
                ' | FaceAlpha = ',num2str(shape.FaceAlpha)];
            line10 = ['EdgeColor = ',mat2str(shape.EdgeColor,2),...
                ' | EdgeAlpha = ',num2str(shape.EdgeAlpha)];
            line11 = ['n = ',num2str(shape.n),' | faces = ',...
                mat2str(shape.faces),];
            
            disp(line1);
            disp(line2);
            disp(line3);
            disp(shape.transform);
            disp(line4);
            disp(line5);
            disp(line6);
            disp(line7);
            disp(line8);
            disp(line9);
            disp(line10);
            disp(line11);
        end
        
        function [shape, frames] = parseopts(shape, varargin)
            %PARSEOPTS Set plot options (and more) in one line
            %
            % Parses multiple plot options to shape properties (faces, 
            % n, FaceColor, FaceAlpha, EdgeColor, EdgeAlpha) in
            % name-value pairs. The note property can also be set.
            % Additionally, the pair "'animate', frames" may be used to
            % animate the shape with the transformations in frames
            % within the plot method. If not specified, is identity.
            %
            % Copyright (C) Bryan Moutrie, 2013-2014
            % Licensed under the GNU General Public License, see file 
            % for statement
            %
            % Syntax:
            %  (1) [shape, frames] = shape.parseopts(...)
            %
            % Outputs:
            %  shape  : Shape object with new property values
            %  frames : Coordinate frames to animate shape. Relative to
            %            the shape's own transform.
            %
            % Inputs:
            %  ... : Options - plotting properties in name-value pairs

            frames = [];
            for i = 1: 2: length(varargin)-1
                if strcmp(varargin{i},'FaceColor')
                    shape.FaceColor = varargin{i+1};
                elseif strcmp(varargin{i},'FaceAlpha')
                    shape.FaceAlpha = varargin{i+1};
                elseif strcmp(varargin{i},'EdgeColor')
                    shape.EdgeColor = varargin{i+1};
                elseif strcmp(varargin{i},'EdgeAlpha')
                    shape.EdgeAlpha = varargin{i+1};
                elseif strcmp(varargin{i},'animate')
                        frames = varargin{i+1};
                elseif strcmp(varargin{i},'n')
                    shape.n = varargin{i+1};
                elseif strcmp(varargin{i},'faces')
                    shape.faces = varargin{i+1};
                elseif strcmp(varargin{i},'note')
                    shape.note = varargin{i+1};
                else
                    error(pHRIWARE('error','inputValue'));
                end
            end
        end
        
        function vargout = animate(shape, h, frames)
            %ANIMATE Animate a plotted shape
            %
            % Animate a plotted shape by applying a series of
            % transformations to its graphics handle. A handle can be 
            % returned by a call to plot. Transformations for each
            % frame are applied before the shape's own transformation.
            % Animation is at a fixed 100 fps.
            %
            % Copyright (C) Bryan Moutrie, 2013-2014
            % Licensed under the GNU General Public License, see file 
            % for statement
            %
            % Syntax:
            %  (1) shape.animate(h, frames)
            %  (2) h = shape.animate(h, frames)
            %
            %  (2) is as per (1) but returns graphics handle of
            %       animated shape
            %
            % Outputs:
            %  h : Modified graphics handle(s)
            %
            % Inputs:
            %  h     : Graphics handle(s) of a Shape object
            %  frames: 4x4xm array of m frames/transformation matrices
            %
            % See also Shape.plot
                        
            t = hgtransform;
            set(h,'Parent',t);
            for i = 1: size(frames,3)
                set(t, 'Matrix', frames(:,:,i));
                pause(0.01);
            end
            
            if nargout == 1, vargout = h; end
        end
             
        function shape = set.note(shape, note)
            if ~ischar(note)
                error(pHRIWARE('error','inputType'));
            else
                shape.note = note;
            end
        end
        
        function shape = set.scale(shape, scale)
            shape.scale = scale .* [1 1 1]; % Allows scalar or 3-vector
        end
        
%         function f = dyncheck(s, points)
%             s.transform = symT * s.transform;
%             switch nargin
%                 case 1
%                     dyncheck = s.check;
%                     f = @(T,x,y,z) dyncheck(T(1),T(5),T(9),...
%                         T(2),T(6),T(10),...
%                         T(3),T(7),T(11),...
%                         T(13),T(14),T(15),x,y,z);
%                 case 2
%                     dyncheck = s.check(points);
%                     f = @(T) dyncheck(T(1),T(5),T(9),...
%                         T(2),T(6),T(10),...
%                         T(3),T(7),T(11),...
%                         T(13),T(14),T(15));
%             end
%         end
    end
    
    methods (Access = protected)
        function [Xst, Yst, Zst] = sat(shape, X, Y, Z)
            %SAT Scale and transform point cloud data
            X_sz = size(X);
            Y_sz = size(Y);
            Z_sz = size(Z);
            
            X = X(:) * shape.scale(1);
            Y = Y(:) * shape.scale(2);
            Z = Z(:) * shape.scale(3);
            
            P = shape.transform * [X, Y, Z, ones(size(X))]';
            Xst = reshape(P(1,:), X_sz);
            Yst = reshape(P(2,:), Y_sz);
            Zst = reshape(P(3,:), Z_sz);
        end
    end
end