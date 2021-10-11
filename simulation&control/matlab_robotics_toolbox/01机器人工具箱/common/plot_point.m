%PLOT_POINT	Draw a feature point
%
% PLOT_POINT(P, OPTIONS) adds point markers to the current plot, where P (2xN)
% and each column is the point coordinate.
%
% Options::
%  'textcolor', colspec     Specify color of text
%  'textsize', size         Specify size of text
%  'bold'                   Text in bold font.
%  'printf', {fmt, data}    Label points according to printf format
%                           string and corresponding element of data
%  'sequence'               Label points sequentially
%
% Additional options are passed through to PLOT for creating the marker.
%
% Examples::
%   Simple point plot
%        P = rand(2,4);
%        plot_point(P);
%
%   Plot points with markers
%        plot_point(P, '*');
%
%   Plot points with square markers and labels 1 to 4
%        plot_point(P, 'sequence', 's');
%
%   Plot points with circles and annotations P1 to P4
%        data = [1 2 4 8];
%        plot_point(P, 'printf', {' P%d', data}, 'o');
%
% Notes::
% - The point(s) is added to the current plot.
% - 2D points only.
%
% See also PLOT, TEXT.

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

function plot_point(p, varargin)

    opt.textcolor = 'g';
    opt.textsize = 12;
    opt.printf = [];
    opt.sequence = false;
    opt.bold = false;

    [opt,arglist] = tb_optparse(opt, varargin);

    % default marker style
    if isempty(arglist)
        arglist = {'sb'};
    end

    % add stuff to pull .u and .v out of a vector of objects
    if ~isnumeric(p) && any(strcmp('u_', properties(p)))
        % p is an object with u_ and v_ properties
        p = [[p.u_]; [p.v_]];
    end

    holdon = ishold();
	hold on
	for i=1:numcols(p)
		plot(p(1,i), p(2,i), arglist{:});
        if opt.sequence
            show(p(:,i), '%d', i, opt);
        end

        if ~isempty(opt.printf)
            show(p(:,i), opt.printf{1}, opt.printf{2}(i), opt);
        end

	end
    if ~holdon
        hold off
    end
    figure(gcf)
end

function show(p, fmt, val, opt)
    if opt.bold
        fw = 'bold';
    else
        fw = 'normal';
    end
    text(p(1), p(2), sprintf([' ' fmt], val), ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle', ...
        'FontUnits', 'pixels', ...
        'FontSize', opt.textsize, ...
        'FontWeight', fw, ...
        'Color', opt.textcolor);
end
