%Ray3D Ray in 3D space
%
% This object represents a ray in 3D space, defined by a point on the ray
% and a direction unit-vector.
%
% Methods::
% intersect    Intersection of ray with plane or ray
% closest      Closest distance between point and ray
% char         Ray parameters as human readable string
% display      Display ray parameters in human readable form
%
% Properties::
% P0    A point on the ray (3x1)
% d     Direction of the ray, unit vector (3x1)
%
% Notes::
% - Ray3D objects can be used in vectors and arrays



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
classdef Ray3D
    properties
        P0   % a point on the ray
        d     % direction of the ray
    end

    methods
        function r = Ray3D(P0, d)
        %Ray3D.Ray3D Ray constructor
        %
        % R = Ray3D(P0, D) is a new Ray3D object defined by a point on the ray P0
        % and a direction vector D.
            r.P0 = P0(:);
            r.d = unit(d(:));
        end

        function [x,e] = intersect(r1, r2)
        %Ray3D.intersect Intersetion of ray with line or plane
        %
        % X = R.intersect(R2) is the point on R that is closest to the ray R2.
        % If R is a vector then then X has multiple columns, corresponding to
        % the intersection of R(i) with R2.
        %
        % [X,E] = R.intersect(R2) as above but also returns the closest distance
        % between the rays.
        %
        % X = R.intersect(P) returns the point of intersection between the
        % ray R and the plane P=(a,b,c,d) where aX + bY + cZ + d = 0.
        % If R is a vector then X has multiple columns, corresponding to
        % the intersection of R(i) with P.

            if isa(r2, 'Ray3D')
                % ray intersect ray case
                if length(r1) ~= length(r2)
                    error('can only intersect rays pairwise');
                end

                for i=1:length(r1)
                    alpha = [-r1(i).d r2(i).d cross(r1(i).d,r2(i).d)] \ ...
                        (r1(i).P0-r2(i).P0);
                    x(:,i) = r1(i).P0 + alpha(1)*r1(i).d;
                    e(i) = norm(r1(i).P0 - r2(i).P0 + ...
                        alpha(1)*r1(i).d -alpha(2)*r2(i).d);
                end
            else
                % ray intersect plane case
                % plane is P = (a,b,c,d), P.x = 0

                for i=1:length(r1)
                    n = r2(1:3); d = r2(4);
                    alpha = -(d + dot(n, r1.P0)) / ( dot(n, r1.d) );
                    x(:,i) = r1.P0 + alpha*r1.d;
                end
            end
        end

        % closest distance between point and line
        function [x,e] = closest(r1, P)
        %Ray3D.closest Closest distance between point and ray
        %
        % X = R.closest(P) is the point on the ray R closest to the point P.
        %
        % [X,E] = R.closest(P) as above but also returns the distance E between
        % X and P.
            alpha = dot(P - r.P0, r.d);
            x = r1.P0 + alpha*r1.d;
            e = norm(x-P);
        end

        function display(rays)
        %Ray3D.display Display value
        %
        % R.display() displays a compact human-readable representation of the Ray3D's
        % value.  If R is a vector then the elements are printed one per line.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a Ray3D object and the command has no trailing
        %   semicolon.
        %
        % See also Ray3D.char.
            disp(' ');
            disp([inputname(1), ' = '])
            disp(' ');
            if length(rays) > 20
                fprintf('%d corresponding points (listing suppressed)\n', length(rays));
            else
                disp( char(rays) );
            end
        end % display()

        function s = char(rays)
        %Ray3D.char Convert to string
        %
        % S = R.char() is a compact string representation of the Ray3D's value.
        % If R is a vector then the string has multiple lines, one per element.

            s = '';
            for r=rays
                ss = sprintf('d=(%g, %g, %g), P0=(%g, %g, %g)\n', ...
                    r.d, r.P0);
                s = strvcat(s, ss);
            end
        end

    end
end
