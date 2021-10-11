%UPQ Central image moments
%
% M = UPQ(IM, P, Q) is the PQ'th central moment of the image IM.  That is, 
% the sum of I(x,y).(x-x0)^P.(y-y0)^Q where (x0,y0) is the centroid.
%
% Notes::
% - The central moments are invariant to translation.
%
% See also UPQ_POLY, MPQ, NPQ.


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

function m = upq(im, p, q)

    [X,Y] = imeshgrid(im);

	m00 = mpq(im, 0, 0);
	xc = mpq(im, 1, 0) / m00;
	yc = mpq(im, 0, 1) / m00;

    m = sum(sum( im.*(X-xc).^p.*(Y-yc).^q ) );
