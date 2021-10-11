%CURVILINEAR Collision checking class for curvilinear prism-type
% primitive shapes
%
% Subclass of the Shape class. The x and y axes of the transform
% describes the radii of the prism's cross-section, and the z axis
% describes the prism's own axis. The translation component describes
% one of the endpoint's of the prism's axis segment, with the other
% endpoint being on the positive z axis of the shape's frame. The scale
% vector defines the radii and the prism length.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) curvilinear = Curvilinear(T, s, radius)
%  (2) curvilinear = Curvilinear(T, s, radius, ...)
%  (3) curvilinear = Curvilinear(P, r, radius, ...)
%  (3) curvilinear = Curvilinear()
%
%  (2) is as per (1) but with other properties set in name-value pairs
%  (3) is as per (1) or (2) but is defined by two points and a radius -
%       the cross-section thus must be circular and not elliptical
%  (4) returns a curvilinear with default values
%
% Outputs:
%  curvilinear : Curvilinear object
%
% Inputs:
%  T      : Transformation matrix. If empty, defaults to eye(4).
%  s      : Scale vector. If empty, defaults to [1 1 1].
%  radius : Radius profile. May be an anonymous function or sym object
%            (which is converted to an anonymous function). Must be a
%            function of one variable, which is the normalised distance
%            along the prism from its origin (i.e. the range should be
%            (0,1), where 0 is at the origin). For cylinders, radius
%            may instead be a scalar number which is scaled by s.
%  P      : Cell, whose first element is the origin, and the second
%            element is the other endpoint. Cell and points may be
%            either column or row vectors.
%  r      : Scalar radius value, for syntax mode (3)
%  ...    : Options - other properties in name-value pairs
%
% See documentation for information on properties and methods
% (type doc Shape into the command window)
%
% See also Box.Box Cone Curvilinear.faces Curvilinear.n
% Curvilinear.profile Cylinder.Cylinder Ellipsoid.Ellipsoid Shape
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

classdef Curvilinear < Shape
    
    properties
        %RADIUS Radius profile
        % Radius profile. May be an anonymous function or sym object
        % which is converted to an anonymous function). Must be a
        % function of one variable, which is the normalised distance
        % along the prism from its origin (i.e. the range should be
        % (0,1), where 0 is at the origin). For cylinders, radius may
        % instead be a scalar number which is scaled by s.
        radius
        
        %FACES  Logical vector of which faces to plot
        % For Curvilinears, faces is a 1x2 vector, whose elements
        %  correspond to the end faces whose centres are at the origin
        %  and not at the origin, with respect to that shape's frame,
        %  in that order
        faces = true(1,2)
        
        %N "Resolution" of the shape, for plotting
        % For Curvilinears, it is the equivalent of cylinder(n)
        n = 10
    end
    
    properties (Dependent)
        volume
    end
    
    methods
        function curvilinear = Curvilinear(T, s, radius, varargin)
            if ~nargin
                T = [];
                s = [1 1 0.4];
                radius = @(t) 0.1 - 0.05*sin(50*t);
            end
            
            if iscell(T) % Syntax mode (3)
                a = T{1};
                b = T{2};
                
                s_z = norm(b-a);
                s = [s s s_z];
                
                T_z = (b(:)-a(:))/s_z;
                T_t = a(:);
                T_x = cross([0; 1; 0], T_z);
                if norm(T_x) < 4*eps
                    T_x = cross([0; 0; -1], T_z);
                end
                T_y = cross(T_z, T_x);
                
                T = [T_x, T_y, T_z, T_t; 0, 0, 0, 1];
            end
                        
            curvilinear = curvilinear@Shape(T, s, varargin{:});
            
            if isnumeric(radius)
                if isscalar(radius)
                    curvilinear.radius = radius;
                else
                    error(pHRIWARE('error', 'inputSize'));
                end
            else
                if isa(radius,'sym')
                    radius = matlabFunction(radius);
                end
                if isa(radius,'function_handle')
                    if nargin(radius) == 1
                        curvilinear.radius = radius;
                    else
                        error(pHRIWARE('error', ...
                            'radius function must have 1 input'));
                    end
                else
                    error(pHRIWARE('error', 'inputType'));
                end
            end
        end
           
        function vargout = plot(curvilinear, varargin)
            [curvilinear, frames] = curvilinear.parseopts(varargin{:});
            
            [X, Y, Z] = curvilinear.cloud;
            
            h = zeros(1,3);
            h(1) = surface(X,Y,Z);
            if curvilinear.faces(1)
                h(2) = patch(X(1,:),Y(1,:),Z(1,:),'b');
            end
            if curvilinear.faces(2)
                h(3) = patch(X(end,:),Y(end,:),Z(end,:),'b');
            end
            h = h([true, curvilinear.faces]);
            
            set(h, ...
                'FaceColor', curvilinear.FaceColor, ...
                'FaceAlpha', curvilinear.FaceAlpha, ...
                'EdgeColor', curvilinear.EdgeColor, ...
                'EdgeAlpha', curvilinear.EdgeAlpha);
            
            if ~isempty(frames), h = curvilinear.animate(h,frames); end  
            if nargout, vargout = h; end
        end
        
        function [X, Y, Z] = cloud(curvilinear)
            if ~isa(curvilinear.radius,'function_handle')
                r = curvilinear.radius;
            else
                r = curvilinear.radius(linspace(0,1,curvilinear.n));
            end
            
            [x, y, z] = cylinder(r);
            [X, Y, Z] = curvilinear.sat(x, y, z);
        end
        
        function inside = check(curvilinear, varargin)
            switch length(varargin)
                case 0
                    if curvilinear.sym == 8
                        syms x y z real
                        points = [x, y, z, 1];
                    else
                        inside = @curvilinear.check;
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
            
            tpts = curvilinear.transform \ points';
            tpts = tpts';
            
            zpts = tpts(:,3);            
            inSegment = 0 < zpts & zpts < curvilinear.scale(3);
            
            if ~isa(curvilinear.radius,'function_handle')
                r1 = curvilinear.scale(1) * curvilinear.radius;
                r2 = curvilinear.scale(2) * curvilinear.radius;
            else
                r1 = curvilinear.scale(1) * ...
                    curvilinear.radius(zpts/curvilinear.scale(3));
                r2 = curvilinear.scale(2) * ...
                    curvilinear.radius(zpts/curvilinear.scale(3));
            end            
            inEllipse = ...
                (tpts(:,1).^2./r1.^2) + (tpts(:,2).^2./r2.^2) < 1;
            
            inside = inSegment & inEllipse;
            
            if isa(inside,'sym'), inside = sym2func(inside); end
        end
        
        function profile(curvilinear, n)
            %PROFILE View how the curvilinear's radius is plotted
            %
            % Depending on the value of n, when plotted, the profile of
            % the curvilinear shape may look inaccurate. This function
            % allows for quick and easy prototyping of a suitable value
            % of n.
            %
            % Copyright (C) Bryan Moutrie, 2013-2014
            % Licensed under the GNU General Public License, see file 
            % for statement
            %
            % Syntax:
            %  (1) curvilinear.profile(n)
            %  (2) curvilinear.profile()
            %
            %  (2) is as per (1) but uses the object's property value
            %
            % Outputs:
            %
            % Inputs:
            %  n : See the Shape property
            %
            % See also Curvilinear.n Curvilinear.plot
            
            if ~isa(curvilinear.radius,'function_handle')
                plot([curvilinear.radius, curvilinear.radius],[0, 1]);
            else
                if nargin == 1, n = curvilinear.n; end
                
                N = linspace(0,1,n);
                hires = 1000;
                Nhr = linspace(0,1,hires);
                
                figure;
                hold on
                plot(curvilinear.radius(N), N, 'o--k');
                plot(curvilinear.radius(Nhr), Nhr);
                hold off
            end
        end
        
        function v = get.volume(s)
            if ~isa(s.radius,'function_handle')
                v = s.radius^2*pi*prod(s.scale);
            else
                t = sym('t');
                r_sq = matlabFunction(s.radius(t)^2);
                v = integral(r_sq,0,1)*pi*prod(s.scale);
            end
        end
        
        function s = set.faces(s, f)
            s.faces = f & [1 1];
        end
    end
    
end

