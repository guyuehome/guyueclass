%MPQ_POLY Polygon moments
%
% M = MPQ_POLY(V, P, Q) is the PQ'th moment of the polygon with vertices 
% described by the columns of V.
%
% Notes::
% - The points must be sorted such that they follow the perimeter in 
%   sequence (counter-clockwise).  
% - If the points are clockwise the moments will all be negated, so centroids
%   will be still be correct.
% - If the first and last point in the list are the same, they are considered
%   to be a single vertex.
%
% See also MPQ, NPQ_POLY, UPQ_POLY, Polygon.



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

function m = mpq(iv, p, q)
    if ~all(iv(:,1) == iv(:,end))
        %disp('closing the polygon')
        iv = [iv iv(:,1)];
    end
    [nr,n] = size(iv);
    if nr < 2,
        error('must be at least two rows of data')
    end
    x = iv(1,:);
    y = iv(2,:);
 
    m = 0.0;
    for l=1:n
        if l == 1
            dxl = x(l) - x(n);
            dyl = y(l) - y(n);
        else
            dxl = x(l) - x(l-1);
            dyl = y(l) - y(l-1);
        end
        Al = x(l)*dyl - y(l)*dxl;
        
        s = 0.0;
        for i=0:p
            for j=0:q
                s = s + (-1)^(i+j) * combin(p,i) * combin(q,j)/(i+j+1) * x(l)^(p-i)*y(l)^(q-j) * dxl^i * dyl^j;
            end
        end
        m = m + Al * s;
    end
    m = m / (p+q+2);

function c = combin(n, r)
% 
% COMBIN(n,r)
%   compute number of combinations of size r from set n
%
    c = prod((n-r+1):n) / prod(1:r);
