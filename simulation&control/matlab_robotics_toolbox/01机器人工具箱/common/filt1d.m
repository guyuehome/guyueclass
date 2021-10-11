%FILT1D    1-dimensional rank filter
%
% Y = FILT1D(X, OPTIONS) is the minimum, maximum or median value (1xN) of the 
% vector X (1xN) compute over an odd length sliding window.
%
% Options::
% 'max'        Compute maximum value over the window (default)
% 'min'        Compute minimum value over the window
% 'median'     Compute minimum value over the window
% 'width',W    Width of the window (default 5)
%
% Notes::
% - If the window width is even, it is incremented by one.
% - The first and last elements of X are replicated so the output vector is the
%   same length as the input vector.


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

% Copyright (c) Peter Corke 6/93
% vectorized version 8/95  pic

function m = filt1d(s, varargin)

    opt.width = 5;
    opt.op = {'max', 'min', 'median'};

    opt = tb_optparse(opt, varargin);

    % enforce a column vector
    s = s(:)';

    % enforce odd window length
    w2 = floor(opt.width/2);
    w = 2*w2 + 1;

    n = length(s);
    m = zeros(w,n+w-1);
    s0 = s(1); sl = s(n);

    % replicate first and last elements
    for i=0:(w-1),
        m(i+1,:) = [s0*ones(1,i) s sl*ones(1,w-i-1)];
    end

    switch (opt.op)
    case 'max'
        m = max(m);
    case 'min'
        m = min(m);
    case 'median'
        m = median(m);
    end

    m = m(w2+1:w2+n);
