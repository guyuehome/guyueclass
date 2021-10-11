%E2H Euclidean to homogeneous
%
% H = E2H(E) is the homogeneous representation of a set of points E.
%
% In the Toolbox points are represented as by Euclidean coordinates which are 
% the columns of a matrix E, and the number of rows is either 2 or 3 to 
% represent 2- or 3-dimensional points.  Homogeous representation increases
% the dimension by one
%
% See also H2E.


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

function h = e2h(e)
    h = [e; ones(1,numcols(e))];
