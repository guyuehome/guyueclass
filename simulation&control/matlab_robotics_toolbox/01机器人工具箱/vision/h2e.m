%H2E Homogeneous to Euclidean 
%
% E = H2E(H) is the Euclidean representation of a set of homogeneous points H.
%
% In the Toolbox points are represented as by Euclidean coordinates which are 
% the columns of a matrix E, and the number of rows is either 2 or 3 to 
% represent 2- or 3-dimensional points.  Euclidean representation decreases
% the dimension of each point by one.
%
% See also E2H.


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

function e = h2e(h)

    if isvec(h)
        h = h(:);
    end
    e = h(1:end-1,:) ./ repmat(h(end,:), numrows(h)-1, 1);

