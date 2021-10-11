%CIRCLE Compute points on a circle
%
% CIRCLE(C, R, OPT) plots a circle centred at C (1x2) with radius R on the current
% axes.
%
% X = CIRCLE(C, R, OPT) is a matrix (2xN) whose columns define the 
% coordinates [x,y] of points around the circumferance of a circle 
% centred at C (1x2) and of radius R.
%
% C is normally 2x1 but if 3x1 then the circle is embedded in 3D, and X is Nx3,
% but the circle is always in the xy-plane with a z-coordinate of C(3).
%
% Options::
%  'n',N   Specify the number of points (default 50)

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
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
%
% http://www.petercorke.com
function out = circle(centre, rad, varargin)

	opt.n = 50;
    
    [opt,arglist] = tb_optparse(opt, varargin);

    % compute points on circumference
	th = [0:opt.n-1]'/ opt.n*2*pi;
    x = rad*cos(th) + centre(1);
    y = rad*sin(th) + centre(2);

    % add extra row if z-coordinate is specified, but circle is always in xy plane
    if length(centre) > 2
        z = ones(size(x))*centre(3);
        p = [x y z]';
    else
        p = [x y]';
    end

    if nargout > 0
        % return now
        out = p;
        return;
    else
        % else plot the circle
        p = [p p(:,1)]; % make it close
        if numrows(p) > 2
            plot3(p(1,:), p(2,:), p(3,:), arglist{:});
        else
            plot(p(1,:), p(2,:), arglist{:});
        end
    end
