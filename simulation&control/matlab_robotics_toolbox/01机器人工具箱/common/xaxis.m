%XAXIS  Set X-axis scaling
%
% XAXIS(MAX) set x-axis scaling from 0 to MAX.
%
% XAXIS(MIN, MAX) set x-axis scaling from MIN to MAX.
%
% XAXIS([MIN MAX]) as above.
%
% XAXIS restore automatic scaling for x-axis.
%
% See also YAXIS.

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

function xaxis(varargin)

    opt.all = false;
    [opt,args] = tb_optparse(opt, varargin);
    
    if length(args) == 0
        [x,y] = ginput(2);
        mn = x(1);
        mx = x(2);
    elseif length(args) == 1
        if length(args{1}) == 1
            mn = 0;
            mx = args{1};
        elseif length(args{1}) == 2
            mn = args{1}(1);
            mx = args{1}(2);
        end
    elseif length(args) == 2
        mn = args{1};
        mx = args{2};
    end

    if opt.all
        for a=get(gcf,'Children')',
            if strcmp(get(a, 'Type'), 'axes') == 1,
                set(a, 'XLimMode', 'manual', 'XLim', [mn mx])
                set(a, 'YLimMode', 'auto')
            end
        end
    else
        set(gca, 'XLimMode', 'manual', 'XLim', [mn mx])
        set(gca, 'YLimMode', 'auto')
    end