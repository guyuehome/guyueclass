%CMFRGB RGB color matching function
%
% The color matching function is the RGB tristimulus required to match a 
% particular spectral excitation.
%
% RGB = CMFRGB(LAMBDA) is the CIE color matching function (Nx3) for illumination
% at wavelength LAMBDA (Nx1) [m].  If LAMBDA is a vector then each row of RGB
% is the color matching function of the corresponding element of LAMBDA. 
%
% RGB = CMFRGB(LAMBDA, E) is the CIE color matching (1x3) function for an 
% illumination spectrum E (Nx1) defined at corresponding wavelengths
% LAMBDA (Nx1).
%
% Notes::
% - Data from http://cvrl.ioo.ucl.ac.uk
% - From Table I(5.5.3) of Wyszecki & Stiles (1982). (Table 1(5.5.3)
%   of Wyszecki & Stiles (1982) gives the Stiles & Burch functions in
%   250 cm-1 steps, while Table I(5.5.3) of Wyszecki & Stiles (1982)
%   gives them in interpolated 1 nm steps.)
% - The Stiles & Burch 2-deg CMFs are based on measurements made on
%   10 observers. The data are referred to as pilot data, but probably
%   represent the best estimate of the 2 deg CMFs, since, unlike the CIE
%   2 deg functions (which were reconstructed from chromaticity data),
%   they were measured directly.
% - These CMFs differ slightly from those of Stiles & Burch (1955). As
%   noted in footnote a on p. 335 of Table 1(5.5.3) of Wyszecki &
%   Stiles (1982), the CMFs have been "corrected in accordance with
%   instructions given by Stiles & Burch (1959)" and renormalized to
%   primaries at 15500 (645.16), 19000 (526.32), and 22500 (444.44) cm-1
%
% References::
%  - Robotics, Vision & Control, Section 10.2,
%    P. Corke, Springer 2011.
%
% See also CMFXYZ, CCXYZ.



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


function rgb = cmfrgb(lambda, spect)
    ciedat = [
  390e-9,  1.83970e-003, -4.53930e-004,  1.21520e-002
  395e-9,  4.61530e-003, -1.04640e-003,  3.11100e-002
  400e-9,  9.62640e-003, -2.16890e-003,  6.23710e-002
  405e-9,  1.89790e-002, -4.43040e-003,  1.31610e-001
  410e-9,  3.08030e-002, -7.20480e-003,  2.27500e-001
  415e-9,  4.24590e-002, -1.25790e-002,  3.58970e-001
  420e-9,  5.16620e-002, -1.66510e-002,  5.23960e-001
  425e-9,  5.28370e-002, -2.12400e-002,  6.85860e-001
  430e-9,  4.42870e-002, -1.99360e-002,  7.96040e-001
  435e-9,  3.22200e-002, -1.60970e-002,  8.94590e-001
  440e-9,  1.47630e-002, -7.34570e-003,  9.63950e-001
  445e-9, -2.33920e-003,  1.36900e-003,  9.98140e-001
  450e-9, -2.91300e-002,  1.96100e-002,  9.18750e-001
  455e-9, -6.06770e-002,  4.34640e-002,  8.24870e-001
  460e-9, -9.62240e-002,  7.09540e-002,  7.85540e-001
  465e-9, -1.37590e-001,  1.10220e-001,  6.67230e-001
  470e-9, -1.74860e-001,  1.50880e-001,  6.10980e-001
  475e-9, -2.12600e-001,  1.97940e-001,  4.88290e-001
  480e-9, -2.37800e-001,  2.40420e-001,  3.61950e-001
  485e-9, -2.56740e-001,  2.79930e-001,  2.66340e-001
  490e-9, -2.77270e-001,  3.33530e-001,  1.95930e-001
  495e-9, -2.91250e-001,  4.05210e-001,  1.47300e-001
  500e-9, -2.95000e-001,  4.90600e-001,  1.07490e-001
  505e-9, -2.97060e-001,  5.96730e-001,  7.67140e-002
  510e-9, -2.67590e-001,  7.01840e-001,  5.02480e-002
  515e-9, -2.17250e-001,  8.08520e-001,  2.87810e-002
  520e-9, -1.47680e-001,  9.10760e-001,  1.33090e-002
  525e-9, -3.51840e-002,  9.84820e-001,  2.11700e-003
  530e-9,  1.06140e-001,  1.03390e+000, -4.15740e-003
  535e-9,  2.59810e-001,  1.05380e+000, -8.30320e-003
  540e-9,  4.19760e-001,  1.05120e+000, -1.21910e-002
  545e-9,  5.92590e-001,  1.04980e+000, -1.40390e-002
  550e-9,  7.90040e-001,  1.03680e+000, -1.46810e-002
  555e-9,  1.00780e+000,  9.98260e-001, -1.49470e-002
  560e-9,  1.22830e+000,  9.37830e-001, -1.46130e-002
  565e-9,  1.47270e+000,  8.80390e-001, -1.37820e-002
  570e-9,  1.74760e+000,  8.28350e-001, -1.26500e-002
  575e-9,  2.02140e+000,  7.46860e-001, -1.13560e-002
  580e-9,  2.27240e+000,  6.49300e-001, -9.93170e-003
  585e-9,  2.48960e+000,  5.63170e-001, -8.41480e-003
  590e-9,  2.67250e+000,  4.76750e-001, -7.02100e-003
  595e-9,  2.80930e+000,  3.84840e-001, -5.74370e-003
  600e-9,  2.87170e+000,  3.00690e-001, -4.27430e-003
  605e-9,  2.85250e+000,  2.28530e-001, -2.91320e-003
  610e-9,  2.76010e+000,  1.65750e-001, -2.26930e-003
  615e-9,  2.59890e+000,  1.13730e-001, -1.99660e-003
  620e-9,  2.37430e+000,  7.46820e-002, -1.50690e-003
  625e-9,  2.10540e+000,  4.65040e-002, -9.38220e-004
  630e-9,  1.81450e+000,  2.63330e-002, -5.53160e-004
  635e-9,  1.52470e+000,  1.27240e-002, -3.16680e-004
  640e-9,  1.25430e+000,  4.50330e-003, -1.43190e-004
  645e-9,  1.00760e+000,  9.66110e-005, -4.08310e-006
  650e-9,  7.86420e-001, -1.96450e-003,  1.10810e-004
  655e-9,  5.96590e-001, -2.63270e-003,  1.91750e-004
  660e-9,  4.43200e-001, -2.62620e-003,  2.26560e-004
  665e-9,  3.24100e-001, -2.30270e-003,  2.15200e-004
  670e-9,  2.34550e-001, -1.87000e-003,  1.63610e-004
  675e-9,  1.68840e-001, -1.44240e-003,  9.71640e-005
  680e-9,  1.20860e-001, -1.07550e-003,  5.10330e-005
  685e-9,  8.58110e-002, -7.90040e-004,  3.52710e-005
  690e-9,  6.02600e-002, -5.67650e-004,  3.12110e-005
  695e-9,  4.14800e-002, -3.92740e-004,  2.45080e-005
  700e-9,  2.81140e-002, -2.62310e-004,  1.65210e-005
  705e-9,  1.91170e-002, -1.75120e-004,  1.11240e-005
  710e-9,  1.33050e-002, -1.21400e-004,  8.69650e-006
  715e-9,  9.40920e-003, -8.57600e-005,  7.43510e-006
  720e-9,  6.51770e-003, -5.76770e-005,  6.10570e-006
  725e-9,  4.53770e-003, -3.90030e-005,  5.02770e-006
  730e-9,  3.17420e-003, -2.65110e-005,  4.12510e-006];

    rgb = interp1(ciedat(:,1), ciedat(:,2:4), lambda, 'spline', 0);
    if nargin == 2,
        rgb = spect(:)' * rgb / numrows(ciedat);
    end
