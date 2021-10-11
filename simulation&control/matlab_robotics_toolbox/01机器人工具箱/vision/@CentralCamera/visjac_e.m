%CentralCamera.visjac_e Visual motion Jacobian for point feature
%
% J = C.visjac_e(E, PL) is the image Jacobian (5x6) for the ellipse
% E (5x1) described by u^2 + E1v^2 - 2E2uv + 2E3u + 2E4v + E5 = 0.  The 
% ellipse lies in the world plane PL = (a,b,c,d) such that aX + bY + cZ + d = 0.
%
% The Jacobian gives the rates of change of the ellipse parameters in 
% terms of camera spatial velocity. 
%
% Reference::
% B. Espiau, F. Chaumette, and P. Rives,
% "A New Approach to Visual Servoing in Robotics",
% IEEE Transactions on Robotics and Automation, 
% vol. 8, pp. 313-326, June 1992.
%
% See also CentralCamera.visjac_p, CentralCamera.visjac_p_polar, CentralCamera.visjac_l.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function L = visjac_e(cam, A, plane)

    a = -plane(1)/plane(4); b = -plane(2)/plane(4); c = -plane(3)/plane(4);
    L = [
2*b*A(2)-2*a*A(1), 2*A(1)*(b-a*A(2)), 2*b*A(4)-2*a*A(1)*A(3), 2*A(4), 2*A(1)*A(3), -2*A(2)*(A(1)+1)
b-a*A(2), b*A(2)-a*(2*A(2)^2-A(1)), a*(A(4)-2*A(2)*A(3))+b*A(3), -A(3), -(2*A(2)*A(3)-A(4)), A(1)-2*A(2)^2-1
c-a*A(3), a*(A(4)-2*A(2)*A(3))+c*A(2), c*A(3)-a*(2*A(3)^2-A(5)), -A(2), 1+2*A(3)^2-A(5), A(4)-2*A(2)*A(3)
A(3)*b+A(2)*c-2*a*A(4), A(4)*b+A(1)*c-2*a*A(2)*A(4), b*A(5)+c*A(4)-2*a*A(3)*A(4), A(5)-A(1), 2*A(3)*A(4)+A(2), -2*A(2)*A(4)-A(3)
2*c*A(3)-2*a*A(5), 2*c*A(4)-2*a*A(2)*A(5), 2*c*A(5)-2*a*A(3)*A(5), -2*A(4), 2*A(3)*A(5)+2*A(3), -2*A(2)*A(5)
];
    L = L * diag([0.5,0.5,0.5, 1,1,1]);   % not sure why...
