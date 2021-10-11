%PLOT_CIRCLE Draw a circle
%
% PLOT_CIRCLE(C, R, options) draws a circle on the current plot with 
% centre C=[X,Y] and radius R.  If C=[X,Y,Z] the circle is drawn in the
% XY-plane at height Z.
%
% H = PLOT_CIRCLE(C, R, options) as above but return handles. For multiple
% circles H is a vector of handles, one per circle.
%
% If C (2xN) then N circles are drawn and H is Nx1.  If R (1x1) then all
% circles have the same radius or else R (1xN) to specify the radius of
% each circle.
%
% Options::
% 'edgecolor'   the color of the circle's edge, Matlab color spec
% 'fillcolor'   the color of the circle's interior, Matlab color spec
% 'alpha'       transparency of the filled circle: 0=transparent, 1=solid
% 'alter',H     alter existing circles with handle H
%
% For an unfilled ellipse any MATLAB LineProperty options can be given, for
% a filled ellipse any MATLAB PatchProperty options can be given.
%
% Notes::
% - The circle(s) is added to the current plot.
%
% See also PLOT_ELLIPSE, PLOT_BOX, PLOT_POLY.



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
function handles = plot_circle(centre, rad, varargin)

    opt.fillcolor = [];
    opt.alpha = 1;
    opt.edgecolor = 'k';
    opt.alter = [];

    [opt,arglist] = tb_optparse(opt, varargin);
    
    if ~isempty(opt.alter) & ~ishandle(opt.alter)
        error('RTB:plot_circle:badarg', 'argument to alter must be a valid graphic object handle');
    end

    holdon = ishold;
    hold on

	n = 50;
	th = [0:n]'/n*2*pi;
    
    if length(rad) == 1
        rad = rad*ones(numcols(centre),1);
    end
    if length(centre) == 2 || length(centre) == 3
        centre = centre(:);
    end

    for i=1:numcols(centre)
        x = rad(i)*cos(th) + centre(1,i);
        y = rad(i)*sin(th) + centre(2,i);
        if numrows(centre) > 2
            % plot 3D data
            z = ones(size(x))*centre(3,i);
            if isempty(opt.alter)
                h(i) = plot3(x, y, z, varargin{:});
            else
                set(opt.alter(i), 'xdata', x, 'ydata', y, 'zdata', z, arglist{:});
            end
        else
            % plot 2D data
            if isempty(opt.fillcolor)
                if isempty(opt.alter)
                    h(i) = plot(x, y, arglist{:});
                else
                    set(opt.alter(i), 'xdata', x, 'ydata', y, arglist{:});
                end
            else
                if isempty(opt.alter)
                    h(i) = patch(x, y, 0*y, 'FaceColor', opt.fillcolor, ...
                        'FaceAlpha', opt.alpha, 'EdgeColor', opt.edgecolor, arglist{:});
                else
                    set(opt.alter(i), 'xdata', x, 'ydata', y, arglist{:});
                end
                
            end
        end
    end

    if holdon == 0
        hold off
    end
    
    if nargout > 0
        handles = h;
    end
