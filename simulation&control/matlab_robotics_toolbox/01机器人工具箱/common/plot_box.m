%PLOT_BOX	Draw a box
%
% PLOT_BOX(B, LS) draws a box defined by B=[XL XR; YL YR] on the current
% plot with optional MATLAB linestyle options LS.
%
% PLOT_BOX(X1,Y1, X2,Y2, LS) draws a box with corners at (X1,Y1) and (X2,Y2),
% and optional MATLAB linestyle options LS.
%
% PLOT_BOX('centre', P, 'size', W, LS) draws a box with center at P=[X,Y] and
% with dimensions W=[WIDTH HEIGHT].
%
% PLOT_BOX('topleft', P, 'size', W, LS) draws a box with top-left at P=[X,Y] 
% and with dimensions W=[WIDTH HEIGHT].
%
% Notes::
% - The box is added to the current plot.
% - Additional options LS are MATLAB LineSpec options and are passed to PLOT.
%
% See also PLOT_POLY, PLOT_CIRCLE, PLOT_ELLIPSE.


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

function plot_box(varargin)
    opt.centre = [];
    opt.topleft = [];
    opt.size = [];

    [opt,varargin] = tb_optparse(opt, varargin);

    if ~isempty(opt.size)
        if size(opt.size) == 1
            w = opt.size;
            h = opt.size;
        else
            w = opt.size(1);
            h = opt.size(2);
        end

        if ~isempty(opt.centre)
            x1 = round(opt.centre(1)-w/2);
            y1 = round(opt.centre(2)-h/2);
            x2 = round(opt.centre(1)+w/2);
            y2 = round(opt.centre(2)+h/2);
        elseif ~isempty(opt.topleft)
            x1 = opt.topleft(1);
            y1 = opt.topleft(2);
            x2 = x1 + w;
            y2 = x1 + h;
        else
            error('must specify top left or centre');
        end
    else
        if all(size(varargin{1}) == [2 2])
            % first arg is a box
            b = varargin{1};
            x1 = b(1); y1 = b(2);
            x2 = b(3); y2 = b(4);
            varargin = varargin(2:end);
        else
            % use first 4 args as x1 y1 x2 y2
            x1 = varargin{1};
            y1 = varargin{2};
            x2 = varargin{3};
            y2 = varargin{4};
            varargin = varargin(5:end);
        end
    end
    p = [	x1 y1
            x2 y1
            x2 y2
            x1 y2
            x1 y1 ];

    holdon = ishold;
    hold on

    plot(p(:,1), p(:,2), varargin{:})

    if holdon == 0
        hold off
    end
