%UPQ_POLY Central polygon moments
%
% M = UPQ_POLY(V, P, Q) is the PQ'th central moment of the polygon with 
% vertices described by the columns of V.
%
% Notes::
% - The points must be sorted such that they follow the perimeter in 
%   sequence (counter-clockwise).  
% - If the points are clockwise the moments will all be negated, so centroids
%   will be still be correct.
% - If the first and last point in the list are the same, they are considered
%   as a single vertex.
% - The central moments are invariant to translation.
%
% See also UPQ, MPQ_POLY, NPQ_POLY.



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

function m = upq_poly(iv, p, q)

	m00 = mpq_poly(iv, 0, 0);
	xc =  mpq_poly(iv, 1, 0) / m00;
	yc =  mpq_poly(iv, 0, 1) / m00;

	m = mpq_poly( bsxfun(@minus, iv, [xc yc]'), p, q);
