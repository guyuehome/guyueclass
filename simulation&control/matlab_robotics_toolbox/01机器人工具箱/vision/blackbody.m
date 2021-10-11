%BLACKBODY Compute blackbody emission spectrum
%
% E = BLACKBODY(LAMBDA, T) is the blackbody radiation power density [W/m^3]
% at the wavelength LAMBDA [m] and temperature T [K].
%
% If LAMBDA is a column vector (Nx1), then E is a column vector (Nx1) of
% blackbody radiation power density at the corresponding elements of LAMBDA.
%
% Example::
%         l = [380:10:700]'*1e-9; % visible spectrum
%         e = blackbody(l, 6500); % emission of sun
%         plot(l, e)
%
% References::
%  - Robotics, Vision & Control, Section 10.1,
%    P. Corke, Springer 2011.



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

function e = blackbody(lam, T)
    % physical constants
    c = 2.99792458e8; % m/s         (speed of light)
    h = 6.626068e-34; % m2 kg / s   (Planck's constant)
    k = 1.3806503e-23; % J K-1      (Boltzmann's constant)

    lam = lam(:);

    e = 2*h*c^2 ./ (lam.^5 .* (exp(h*c/k/T./lam) - 1));
