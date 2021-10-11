
% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
classdef scamera
    properties
        name
        Tcam
    end


    methods
        function c = scamera
            c.Tcam = eye(4,4);
        end


        function f = plot(c, points, varargin)
            f = c.project(points, varargin{:});
            plot(f(2,:), f(1,:), 'o');
            xlabel('phi (rad)');
            ylabel('theta (rad)');
            grid on
            axis([-pi pi 0 pi]);
        end

        function f = project(c, P, Tcam)
            if nargin < 3
                Tcam = c.Tcam;
            end
            P = transformp(inv(Tcam), P);

            R = sqrt( sum(P.^2) );
            x = P(1,:) ./ R;
            y = P(2,:) ./ R;
            z = P(3,:) ./ R;
            r = sqrt( x.^2 + y.^2);
            theta = atan2(r, z);
            phi = atan2(y, x);
            f = [theta; phi];
        end
    end
end
