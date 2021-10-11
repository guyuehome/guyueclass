%CLOSEST Find closest points in N-dimensional space.
%
% K = CLOSEST(A, B) is the correspondence for N-dimensional point sets A (NxNA)
% and B (NxNB).  K (1 x NA) is such that the element J = K(I), that is, that 
% the I'th column of A is closest to the Jth column of B.
%
% [K,D1] = CLOSEST(A, B) as above and D1(I)=|A(I)-B(J)| is the distance of the
% closest point.
%
% [K,D1,D2] = CLOSEST(A, B) as above but also returns the distance to the
% second closest point.
%
% Notes::
% - Is a MEX file.
%
% See also DISTANCE.


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
% - is a MEX file.

if ~exist('closest', 'file')
    error('you need to build the MEX version of closest, see vision/mex/README');
end
