%EPILINE Draw epipolar lines
%
% EPILINE(F, P) draws epipolar lines in current figure based on points P (2xN)
% and the fundamental matrix F (3x3).  Points are specified by the columns of P.
%
% EPILINE(F, P, LS) as above but draw lines using the line style arguments LS.
%
% H = EPILINE(F, P, LS) as above but return a vector of graphic handles, one
% per line drawn.
% 
% See also FMATRIX, EPIDIST.



% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

function handles = epiline(F, p, ls)

    % get plot limits from current graph
    xlim = get(gca, 'XLim');
    xmin = xlim(1);
    xmax = xlim(2);

    if nargin < 3,
        ls = 'r';
    end
    h = [];
    % for all input points
    for i=1:numrows(p),
        l = F*[p(i,:) 1]';
        y = (-l(3) - l(1)*xlim) / l(2);
        hold on
        hh = plot(xlim, y, ls);
        h = [h; hh];
        hold off
    end

    if nargout > 0,
        handles = h;
    end
