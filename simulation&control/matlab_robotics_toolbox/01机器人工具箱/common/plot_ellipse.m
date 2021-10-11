%PLOT_ELLIPSE Draw an ellipse or ellipsoid
%
% PLOT_ELLIPSE(A, OPTIONS) draws an ellipse defined by X'AX = 0 on the
% current plot, centred at the origin.
%
% PLOT_ELLIPSE(A, C, OPTIONS) as above but centred at C=[X,Y].  If
% C=[X,Y,Z] the ellipse is parallel to the XY plane but at height Z.
%
% H = PLOT_ELLIPSE(A, C, OPTIONS) as above but return graphic handle.
%
% Options::
% 'edgecolor'   the color of the circle's edge, Matlab color spec
% 'fillcolor'   the color of the circle's interior, Matlab color spec
% 'alpha'       transparency of the filled circle: 0=transparent, 1=solid
% 'alter',H     alter existing circles with handle H
%
% Notes::
% - If A (2x2) draw an ellipse, else if A(3x3) draw an ellipsoid.
% - The ellipse is added to the current plot.
%
% See also PLOT_ELLIPSE_INV, PLOT_CIRCLE, PLOT_BOX, PLOT_POLY.

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

function handles = plot_ellipse(A, centre, varargin)
    
    if size(A,1) ~= size(A,2)
        error('ellipse is defined by a square matrix');
    end
    
    if size(A,1) > 3
        error('can only plot ellipsoid for 2 or 3 dimenions');
    end
    
    if nargin < 2
        centre = zeros(1, size(A,1));
    end
    if nargin < 3
        varargin = {};
    end
    
    opt.fillcolor = 'none';
    opt.alpha = 1;
    opt.edgecolor = 'k';
    opt.alter = [];
    
    [opt,arglist,ls] = tb_optparse(opt, varargin);

    if ~isempty(ls)
        opt.edgecolor = ls;
    end
    
    if ~isempty(opt.alter) & ~ishandle(opt.alter)
        error('RTB:plot_circle:badarg', 'argument to alter must be a valid graphic object handle');
    end
    
    holdon = ishold();
    hold on
    
    if size(A,1) == 3
        %% plot an ellipsoid
        
        % define mesh points on the surface of a unit sphere
        [Xs,Ys,Zs] = sphere();
        ps = [Xs(:) Ys(:) Zs(:)]';
        
        % warp it into the ellipsoid
        pe = sqrtm(A) * ps;
        
        % offset it to optional non-zero centre point
        if nargin > 1
            pe = bsxfun(@plus, centre(:), pe);
        end
        
        % put back to mesh format
        Xe = reshape(pe(1,:), size(Xs));
        Ye = reshape(pe(2,:), size(Ys));
        Ze = reshape(pe(3,:), size(Zs));
        
        % plot it
        if isempty(opt.alter)
            h = mesh(Xe, Ye, Ze, 'FaceColor', opt.fillcolor, ...
                        'FaceAlpha', opt.alpha, 'EdgeColor', opt.edgecolor, arglist{:});
        else
            set(opt.alter, 'xdata', Xe, 'ydata', Ye, 'zdata', Ze,  ...
                        arglist{:});
            
        end
        
    else
        %% plot an ellipse
        
        
        [V,D] = eig(A);
        
        % define points on a unit circle
        th = linspace(0, 2*pi, 50);
        pc = [cos(th);sin(th)];
        
        % warp it into the ellipse
        pe = sqrtm(A)*pc;
        
        % offset it to optional non-zero centre point
        centre = centre(:);
        if nargin > 1
            pe = bsxfun(@plus, centre(1:2), pe);
        end
        x = pe(1,:); y = pe(2,:);
        

        if length(centre) > 2
            % plot 3D data
            z = ones(size(x))*centre(3);
            if isempty(opt.alter)
                h = plot3(x', y', z', varargin{:});
            else
                set(opt.alter, 'xdata', x, 'ydata', y, 'zdata', z, arglist{:});
            end
        else
            % plot 2D data
            if isempty(opt.fillcolor)
                % outline only, draw a line
                if isempty(opt.alter)
                    h = plot(x', y', 'Color', opt.edgecolor, arglist{:});
                else
                    set(opt.alter, 'xdata', x, 'ydata', y, arglist{:});
                end
            else
                % filled, use a patch
                if isempty(opt.alter)
                    h = patch(x', y', 0*y, 'FaceColor', opt.fillcolor, ...
                        'FaceAlpha', opt.alpha, 'EdgeColor', opt.edgecolor, arglist{:});
                else
                    set(opt.alter, 'xdata', x, 'ydata', y, 'FaceColor', opt.fillcolor, ...
                        'FaceAlpha', opt.alpha, 'EdgeColor', opt.edgecolor, arglist{:});
                end
                
            end
        end
    end
    holdon = ishold;
    hold on
    
    if nargout > 0
        handles = h;
    end
end
