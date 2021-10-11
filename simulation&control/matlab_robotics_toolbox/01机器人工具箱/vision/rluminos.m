%RLUMINOS Relative photopic luminosity function
%
% P = RLUMINOS(LAMBDA) is the relative photopic luminosity function for the 
% wavelengths in LAMBDA [m].  If LAMBDA is a vector (Nx1), then P (Nx1) is a
% vector whose elements are the luminosity at the corresponding elements 
% of LAMBDA.
%
% Relative luminosity lies in the interval 0 to 1 which indicate the intensity 
% with which wavelengths are perceived by the light-adapted human eye.
%
% References::
%  - Robotics, Vision & Control, Section 10.1,
%    P. Corke, Springer 2011.
%
% See also LUMINOS.



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
function lu = rluminos(lambda)
    xyz = cmfxyz(lambda);
    lu = xyz(:,2);  % photopic luminosity is the Y color matching function
