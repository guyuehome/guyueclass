

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
function sphrot = sphere_rotate(sph, T, varargin)
    % if only a hemisphere pad the other hemisphere with grey
    if nargin > 2
        if strcmp(varargin(1), 'north')
            sph = [sph; 0.3*ones(size(sph))];
        elseif strcmp(varargin(1), 'south')
            sph = [0.3*ones(size(sph)); sph];
        end
    end

    [nr,nc] = size(sph);
    nr2 = floor(nr/2);

    % theta spans [0, pi]
    theta_range = (0:nr-1)/(nr-1)*pi;

    % phi spans [-pi, pi]
    phi_range = ((0:nc-1)/(nc-1) - 0.5)*2*pi;

    % build the plaid matrices
    [Phi,Theta] = meshgrid(phi_range, theta_range);

    % convert the spherical coordinates to Cartesian
    r = sin(Theta);
    x = r .* cos(Phi);
    y = r .* sin(Phi);
    z = cos(Theta);

    % convert to 3xN format
    p = [x(:)' ; y(:)'; z(:)'];

    % transform the points
    p = homtrans(T, p);

    % convert back to Cartesian coordinate matrices
    x = reshape(p(1,:), size(x));
    y = reshape(p(2,:), size(x));
    z = reshape(p(3,:), size(x));

    % convert back to spherical coordinates
    r = sqrt(x.^2 + y.^2);

    % asin is multiple valued over the interval [0,pi]
    nTheta = asin(r);
    nTheta(nr2:end,:) = pi-nTheta(nr2:end,:);

    nPhi = atan2(y, x);

    whos
    % warp the image
    sphrot = interp2(Phi, Theta, sph, nPhi, nTheta);
