%PLOTP Plot trajectories
%
% PLOTP(P) plots a set of points P, which by Toolbox convention are stored
% one per column.  P can be Nx2 or Nx3.  By default a linestyle of 'bx'
% is used.
%
% PLOTP(P, LS) as above but the line style arguments LS are passed to plot.
%
% See also PLOT, PLOT2.

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
function h = plotp(p1, varargin)

    if length(varargin) == 0
        varargin = {'bx'};
    end

    if numrows(p1) == 3,
        hh = plot3(p1(1,:), p1(2,:), p1(3,:), varargin{:});
        xyzlabel
    else
        hh = plot(p1(1,:), p1(2,:), varargin{:});
        xlabel('x');
        ylabel('y');
    end
    if nargout == 1,
        h = hh;
    end
