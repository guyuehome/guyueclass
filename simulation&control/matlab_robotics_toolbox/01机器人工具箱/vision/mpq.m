%MPQ Image moments
%
% M = MPQ(IM, P, Q) is the PQ'th moment of the image IM.  That is, the sum
% of I(x,y).x^P.y^Q.
%
% See also MPQ_POLY, NPQ, UPQ.


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

function m = mpq(im, p, q)

    [X,Y] = imeshgrid(im);

    m = sum(sum( im.*X.^p.*Y.^q ) );
