% XY = LAMBDA2XY(LAMBDA) is the xy-chromaticity coordinate (1x2) for 
% illumination at the specific wavelength LAMBDA [metres]. If LAMBDA is a
% vector (Nx1), then P (Nx2) is a vector whose elements are the luminosity 
% at the corresponding elements of LAMBDA.
%
% XY = LAMBDA2XY(LAMBDA, E) is the rg-chromaticity coordinate (1x2) for an 
% illumination spectrum E (Nx1) defined at corresponding wavelengths
% LAMBDA (Nx1).
%
% References::
%  - Robotics, Vision & Control, Section 10.2,
%    P. Corke, Springer 2011.
%
% See also CMFXYZ, LAMBDA2RG.


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
function [x,y] = lambda2xy(lambda, varargin)
    cmf = cmfxyz(lambda, varargin{:});

    xy = tristim2cc(cmf);
    if nargout == 2
        x = xy(1);
        y = xy(2);
    else
        x = xy;
    end

