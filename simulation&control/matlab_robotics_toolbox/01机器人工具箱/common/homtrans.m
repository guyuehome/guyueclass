%HOMTRANS Apply a homogeneous transformation
%
% P2 = HOMTRANS(T, P) applies homogeneous transformation T to the points 
% stored columnwise in P.
%
% - If T is in SE(2) (3x3) and
%   - P is 2xN (2D points) they are considered Euclidean (R^2)
%   - P is 3xN (2D points) they are considered projective (P^2)
% - If T is in SE(3) (4x4) and
%   - P is 3xN (3D points) they are considered Euclidean (R^3)
%   - P is 4xN (3D points) they are considered projective (P^3)
%
% TP = HOMTRANS(T, T1) applies homogeneous transformation T to the 
% homogeneous transformation T1, that is TP=T*T1.  If T1 is a 3-dimensional 
% transformation then T is applied to each plane as defined by the first two 
% dimensions, ie. if T = NxN and T=NxNxP then the result is NxNxP.
%
% See also E2H, H2E.


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
function pt = homtrans(T, p)

    if numrows(p) == numrows(T)
        if ndims(p) == 3
            pt = [];
            for i=1:size(p,3)
                pt = cat(3, pt, T*p(:,:,i));
            end
        else
            pt = T * p;
        end
    elseif (numrows(T)-numrows(p)) == 1
        % second argument is Euclidean coordinate, promote to homogeneous
        pt = h2e( T * e2h(p) );
    else
        error('matrices and point data do not conform')
    end
