%GAUSS1	Gaussian kernel
%
%	k = gauss1(, c, sigma)
%
%	Returns a unit volume Gaussian smoothing kernel.  The Gaussian has 
%	a standard deviation of sigma, and the convolution
%	kernel has a half size of w, that is, k is (2W+1) x (2W+1).
%

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
function g = gaussfunc(c, var, x, y)

    if length(c) == 1
        g = 1/sqrt(2*pi*var) * exp( -((x-c).^2)/(2*var) );
    elseif length(c) == 2
        if nargin < 4
            [x,y] = imeshgrid(x);
        end
        xx = x(:)-c(1); yy = y(:)-c(2);
        ci = inv(var);
        g = 1/(2*pi*sqrt(det(var))) * exp( -0.5*(xx.^2*ci(1,1) + yy.^2*ci(2,2) + 2*xx.*yy*ci(1,2)));
        g = reshape(g, size(x));
    end
