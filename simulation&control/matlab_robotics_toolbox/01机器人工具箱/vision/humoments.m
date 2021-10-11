%HUMOMENTS Hu moments
%
% PHI = HUMOMENTS(IM) is the vector (7x1) of Hu moment invariants for the binary
% image IM.
%
% Notes::
% - IM is assumed to be a binary image of a single connected region
%
% Reference::
% M-K. Hu, 
% Visual pattern recognition by moment invariants. 
% IRE Trans. on Information Theory, IT-8:pp. 179-187, 1962.
%
% See also NPQ.


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

function phi = humoments(im)

    % second moments
    eta_20 = npq(im, 2, 0);
    eta_02 = npq(im, 0, 2);
    eta_11 = npq(im, 1, 1);

    % third moments
    eta_30 = npq(im, 3, 0);
    eta_03 = npq(im, 0, 3);
    eta_21 = npq(im, 2, 1);
    eta_12 = npq(im, 1, 2);

    phi(1) = eta_20 + eta_02;
    phi(2) = (eta_20 - eta_02)^2 + 4*eta_11^2;
    phi(3) = (eta_30 - 3*eta_12)^2 + (3*eta_21 - eta_03)^2;
    phi(4) = (eta_30 + eta_12)^2 + (eta_21 + eta_03)^2;
    phi(5) = (eta_30 - 3*eta_12)*(eta_30+eta_12)* ...
       ((eta_30 +eta_12)^2 - 3*(eta_21+eta_03)^2) + ...
       (3*eta_21 - eta_03)*(eta_21+eta_03)* ...
       (3*(eta_30+eta_12)^2 - (eta_21+eta_03)^2);
    phi(6) = (eta_20 - eta_02)*((eta_30 +eta_12)^2 - (eta_21+eta_03)^2) + ...
       4*eta_11 *(eta_30+eta_12)*(eta_21+eta_03);
    phi(7) = (3*eta_21 - eta_03)*(eta_30+eta_12)* ...
      ((eta_30 +eta_12)^2 - 3*(eta_21+eta_03)^2) + ...
      (3*eta_12 - eta_30)*(eta_21+eta_03)*( 3*(eta_30+eta_12)^2 - (eta_21+eta_03)^2);

