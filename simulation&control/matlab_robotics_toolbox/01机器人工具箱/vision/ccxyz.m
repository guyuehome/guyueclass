%CCXYZ XYZ chromaticity coordinates
%
% XYZ = CCXYZ(LAMBDA) is the xyz-chromaticity coordinates (3x1) for illumination
% at wavelength LAMBDA.  If LAMBDA is a vector (Nx1) then each row of XYZ (Nx3)
% is the XYZ-chromaticity of the corresponding element of LAMBDA. 
%
% XYZ = CCXYZ(LAMBDA, E) is the xyz-chromaticity coordinates (Nx3) for an 
% illumination spectrum E (Nx1) defined at corresponding wavelengths
% LAMBDA (Nx1).
%
% References::
%  - Robotics, Vision & Control, Section 10.2,
%    P. Corke, Springer 2011.
%
% See also CMFXYZ.



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
function [x,y] = ccxyz(lambda, e)
    xyz = cmfxyz(lambda);
    if nargin == 1,
        cc = xyz ./ (sum(xyz')'*ones(1,3));
    elseif nargin == 2,
        xyz = xyz .* (e(:)*ones(1,3));
        xyz = sum(xyz);
        cc = xyz ./ (sum(xyz')'*ones(1,3));
    end

    if nargout == 1
        x = cc;
    elseif nargout == 2
        x = cc(:,1);
        y = cc(:,2);
    end
