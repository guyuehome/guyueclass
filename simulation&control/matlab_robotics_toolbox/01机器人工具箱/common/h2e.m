%H2E Homogeneous to Euclidean 
%
% E = H2E(H) is the Euclidean version (K-1xN) of the homogeneous 
% points H (KxN) where each column represents one point in P^K.
%
% See also E2H.

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

function e = h2e(h)

    if isvector(h)
        h = h(:);
    end
    e = h(1:end-1,:) ./ repmat(h(end,:), numrows(h)-1, 1);

