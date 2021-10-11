%LUMINOS Photopic luminosity function
%
% P = LUMINOS(LAMBDA) is the photopic luminosity function for the wavelengths
% in LAMBDA [m].  If LAMBDA is a vector (Nx1), then P (Nx1) is a vector whose 
% elements are the luminosity at the corresponding elements of LAMBDA.
%
% Luminosity has units of lumens which are the intensity with 
% which wavelengths are perceived by the light-adapted human eye.
%
% References::
%  - Robotics, Vision & Control, Section 10.1,
%    P. Corke, Springer 2011.
%
% See also RLUMINOS.


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
function lu = luminos(lam)
      tab = [
      3.8000000e-007  0.0000000e+000
      3.8500000e-007  1.0000000e-004
      3.9000000e-007  1.0000000e-004
      3.9500000e-007  2.0000000e-004
      4.0000000e-007  4.0000000e-004
      4.0500000e-007  6.0000000e-004
      4.1000000e-007  1.2000000e-003
      4.1500000e-007  2.2000000e-003
      4.2000000e-007  4.0000000e-003
      4.2500000e-007  7.3000000e-003
      4.3000000e-007  1.1600000e-002
      4.3500000e-007  1.6800000e-002
      4.4000000e-007  2.3000000e-002
      4.4500000e-007  2.9800000e-002
      4.5000000e-007  3.8000000e-002
      4.5500000e-007  4.8000000e-002
      4.6000000e-007  6.0000000e-002
      4.6500000e-007  7.3900000e-002
      4.7000000e-007  9.1000000e-002
      4.7500000e-007  1.1260000e-001
      4.8000000e-007  1.3900000e-001
      4.8500000e-007  1.6930000e-001
      4.9000000e-007  2.0800000e-001
      4.9500000e-007  2.5860000e-001
      5.0000000e-007  3.2300000e-001
      5.0500000e-007  4.0730000e-001
      5.1000000e-007  5.0300000e-001
      5.1500000e-007  6.0820000e-001
      5.2000000e-007  7.1000000e-001
      5.2500000e-007  7.9320000e-001
      5.3000000e-007  8.6200000e-001
      5.3500000e-007  9.1490000e-001
      5.4000000e-007  9.5400000e-001
      5.4500000e-007  9.8030000e-001
      5.5000000e-007  9.9500000e-001
      5.5500000e-007  1.0002000e+000
      5.6000000e-007  9.9500000e-001
      5.6500000e-007  9.7860000e-001
      5.7000000e-007  9.5200000e-001
      5.7500000e-007  9.1540000e-001
      5.8000000e-007  8.7000000e-001
      5.8500000e-007  8.1630000e-001
      5.9000000e-007  7.5700000e-001
      5.9500000e-007  6.9490000e-001
      6.0000000e-007  6.3100000e-001
      6.0500000e-007  5.6680000e-001
      6.1000000e-007  5.0300000e-001
      6.1500000e-007  4.4120000e-001
      6.2000000e-007  3.8100000e-001
      6.2500000e-007  3.2100000e-001
      6.3000000e-007  2.6500000e-001
      6.3500000e-007  2.1700000e-001
      6.4000000e-007  1.7500000e-001
      6.4500000e-007  1.3820000e-001
      6.5000000e-007  1.0700000e-001
      6.5500000e-007  8.1600000e-002
      6.6000000e-007  6.1000000e-002
      6.6500000e-007  4.4600000e-002
      6.7000000e-007  3.2000000e-002
      6.7500000e-007  2.3200000e-002
      6.8000000e-007  1.7000000e-002
      6.8500000e-007  1.1900000e-002
      6.9000000e-007  8.2000000e-003
      6.9500000e-007  5.7000000e-003
      7.0000000e-007  4.1000000e-003
      7.0500000e-007  2.9000000e-003
      7.1000000e-007  2.1000000e-003
      7.1500000e-007  1.5000000e-003
      7.2000000e-007  1.0000000e-003
      7.2500000e-007  7.0000000e-004
      7.3000000e-007  5.0000000e-004
      7.3500000e-007  4.0000000e-004
      7.4000000e-007  3.0000000e-004
      7.4500000e-007  2.0000000e-004
      7.5000000e-007  1.0000000e-004
      7.5500000e-007  1.0000000e-004
      7.6000000e-007  1.0000000e-004
      7.6500000e-007  0.0000000e+000
      7.7000000e-007  0.0000000e+000];
        
    lu = [];

    for lmd = lam(:)',
        if (lmd < 380e-9) | (lmd > 770e-9)
            lu = [lu; 0.0];
        else
            lu = [lu; 683*interp1(tab(:,1), tab(:,2), lmd)];
        end
    end;
