%PEAK Find peaks in vector
%
% YP = PEAK(Y, OPTIONS) are the values of the maxima in the vector Y.
%
% [YP,I] = PEAK(Y, OPTIONS) as above but also returns the indices of the maxima
% in the vector Y.
%
% [YP,XP] = PEAK(Y, X, OPTIONS) as above but also returns the corresponding 
% x-coordinates of the maxima in the vector Y.  X is the same length as Y
% and contains the corresponding x-coordinates.
%
% Options::
% 'npeaks',N    Number of peaks to return (default all)
% 'scale',S     Only consider as peaks the largest value in the horizontal 
%               range +/- S points.
% 'interp',M    Order of interpolation polynomial (default no interpolation)
% 'plot'        Display the interpolation polynomial overlaid on the point data
%
% Notes::
% - A maxima is defined as an element that larger than its two neighbours.
%   The first and last element will never be returned as maxima.
% - To find minima, use PEAK(-V).
% - The interp options fits points in the neighbourhood about the peak with
%   an M'th order polynomial and its peak position is returned.  Typically
%   choose M to be odd.  In this case XP will be non-integer.
%
% See also PEAK2.



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

% Copyright (c) Peter Corke 1/96

function [yp,xpout] = peak(y, varargin)

    % process input options
    opt.npeaks = [];
    opt.scale = 1;
    opt.interp = 0;
    opt.plot = false;
    
    [opt,args] = tb_optparse(opt, varargin);
    
    
    % if second argument is a matrix we take this as the corresponding x
    % coordinates
    if ~isempty(args)
        x = args{1};
        x = x(:);
        if length(x) ~= length(y)
            error('second argument must be same length as first');
        end
    else
        x = [1:length(y)]';
    end
    
    y = y(:);
    
    % find the maxima
    if opt.scale > 1
        % compare to a moving window max filtered version
        k = find(y' == filt1d(y, 'max', 'width', opt.scale*2+1));
    else
        % take the zero crossings
        dv = diff(y);
        k = find( ([dv; 0]<0) & ([0; dv]>0) );
    end
    
    % sort the maxima into descending magnitude
    [m,i] = sort(y(k), 'descend');
    k = k(i);    % indice of the maxima

    if opt.npeaks
        np = min(length(k), opt.npeaks);
        k = k(1:np);
    end

    % optionally plot the discrete data
    if opt.plot
        plot(x, y, '-o');      
        hold on
    end
    

    % interpolate the peaks if required
    if opt.interp
        if opt.interp < 2
            error('interpolation polynomial must be at least second order');
        end
        
        xp = [];
        yp = [];
        N = opt.interp;
        N2 = round(N/2);

        % for each previously identified peak x(i), y(i)
        for i=k'
            % fit a polynomial to the local neighbourhood
            try
                pp = polyfit(x(i-N2:i+N2), y(i-N2:i+N2), N);
            catch
                % handle situation where neighbourhood falls off the data
                % vector
                warning('Peak at %f too close to start or finish of data, skipping', x(i));
                continue;
            end
            
            % find the roots of the polynomial closest to the coarse peak
            r = roots( polydiff(pp) );
            [mm,j] = min(abs(r-x(i)));
            xx = r(j);
            
            % store x, y for the refined peak
            xp = [xp; xx];
            yp = [y; polyval(pp, xx)];
            
            if opt.plot
                % overlay the fitted polynomial and refined peak
                xr = linspace(x(i-N2), x(i+N2), 50);
                plot(xr, polyval(pp, xr), 'r');
                plot(xx, polyval(pp, xx), 'rd');
            end
        end
    else
        xp = x(k);
    end
    
    if opt.plot
        grid
        xlabel('x');
        ylabel('y');
        hold off
    end
    
    % return values
    yp = y(k)';
    if nargout > 1
        xpout = xp';
    end
