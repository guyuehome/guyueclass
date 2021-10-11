%PLOT_POLY Draw a polygon
%
% PLOT_POLY(P, OPTIONS) draws a polygon defined by columns of P (2xN), in the current plot.
%
% OPTIONS::
%  'fill',F    the color of the circle's interior, MATLAB color spec
%  'alpha',A   transparency of the filled circle: 0=transparent, 1=solid.
%
% Notes::
% - If P (3xN) the polygon is drawn in 3D
% - The line(s) is added to the current plot.
%
% See also PLOT_BOX, PATCH, Polygon.

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

% TODO: options for fill, not filled, line style, labels (cell array of strings)

function h_ = plot_poly(p, varargin)

    if numcols(p) < 3,
        error('too few points for a polygon');
    end

    opt.fill = [];
    opt.alpha = 1;

    [opt,arglist] = tb_optparse(opt, varargin);

    % default marker style
    if isempty(arglist)
        arglist = {'r-'};
    end

    ish = ishold();
	hold on

    x = [p(1,:) p(1,1)];
    y = [p(2,:) p(2,1)];
    if numrows(p) == 2
        % plot 2D data
        h(1) = plot(x, y, arglist{:});
        if ~isempty(opt.fill)
            h(2) = patch(x', y', 0*y', 'FaceColor', opt.fill, ...
                'FaceAlpha', opt.alpha);
        end
    elseif numrows(p) == 3
        % plot 3D data
        z = [p(3,:) p(3,1)];
        h(1) = plot3(x, y, z, arglist{:});
        if ~isempty(opt.fill)
            h(2) = patch(x, y, z, 0*y, 'FaceColor', opt.fill, ...
                'FaceAlpha', opt.alpha);
        end
    else
        error('point data must have 2 or 3 rows');
    end

    if ~ish
        hold off
    end
    %figure(gcf)
    
    if nargout > 0
        h_ = h;
    end
