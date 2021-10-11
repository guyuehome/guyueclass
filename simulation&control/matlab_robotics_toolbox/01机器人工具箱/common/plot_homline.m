%PLOT_HOMLINE Draw a homogeneous
%
% PLOT_HOMLINE(L, LS) draws a line in the current plot L.X = 0 where L
% (3x1).  The current axis limits are used to determine the endpoints of
% the line.  MATLAB line specification LS can be set.  If L (3xN) then N
% lines are drawn, one per column.
%
% H = PLOT_HOMLINE(L, LS) as above but returns a vector of graphics handles for the lines.
%
% Notes::
% - The line(s) is added to the current plot.
%
% See also PLOT_BOX, PLOT_POLY, HOMLINE.

% TODO, probably should be part of a HomLine class.


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

function handles = plot_homline(lines, varargin)

	% get plot limits from current graph
	xlim = get(gca, 'XLim');
	ylim = get(gca, 'YLim');

    ish = ishold;
    hold on;

	h = [];
	% for all input lines (columns
	for l=lines
        if abs(l(2)) > abs(l(1))
            y = (-l(3) - l(1)*xlim) / l(2);
            hh = plot(xlim, y, varargin{:});
        else
            x = (-l(3) - l(2)*ylim) / l(1);
            hh = plot(x, ylim, varargin{:});
        end
		h = [h; hh];
	end

    if ~ish
        hold off
    end

	if nargout > 0,
		handles = h;
	end
