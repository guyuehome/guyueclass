%GAUSS2D    Gaussian kernel
%
% OUT = GAUSS2D(IM, SIGMA, C) is a unit volume Gaussian kernel rendered into
% matrix OUT (WxH) the same size as IM (WxH). The Gaussian has a standard 
% deviation of SIGMA.  The Gaussian is centered at C=[U,V].

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
function m = gaus2d(im, sigma, c)


    if length(sigma) == 1
        sx = sigma(1);
        sy = sigma(1);
    else
        sx = sigma(1);
        sy = sigma(2);
    end

    [x,y] = imeshgrid(im);

    m = 1/(2*pi*sx*sy) * exp( -(((x-c(1))/sx).^2 + ((y-c(2))/sy).^2)/2);
