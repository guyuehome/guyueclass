%SphericalCamera.visjac_p Visual motion Jacobian for point feature
%
% J = C.visjac_p(PT, Z) is the image Jacobian (2Nx6) for the image plane 
% points PT (2xN) described by phi (longitude) and theta (colatitude).
% The depth of the points from the camera is given by Z which is a scalar,
% for all points, or a vector (Nx1) for each point.
%
% The Jacobian gives the image-plane velocity in terms of camera spatial
% velocity. 
%
% Reference::
%
% "Spherical image-based visual servo and structure estimation",
% P. I. Corke, 
% in Proc. IEEE Int. Conf. Robotics and Automation, (Anchorage),
% pp. 5550-5555, May 3-7 2010.
%
% See also CentralCamera.visjac_p_polar, CentralCamera.visjac_l, CentralCamera.visjac_e.

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

function L = visjac_p(cam, p, R)

    if numcols(p) > 1
        L = [];
        if length(R) == 1
            % if depth is a scalar, assume same for all points
            R = repmat(R, 1, numcols(p));
        end
        % recurse for each point
        for i=1:numcols(p)
            L = [L; cam.visjac_p(p(:,i), R(i))];
        end
        return;
    end
    
    phi = p(1); theta = p(2); 
    cp = cos(phi); sp = sin(phi);
    ct = cos(theta); st = sin(theta);

    L = [ sp/R/st  -cp/R/st  0     cp*ct/st  sp*ct/st -1
         -cp*ct/R  -sp*ct/R  st/R  sp       -cp        0  ];
