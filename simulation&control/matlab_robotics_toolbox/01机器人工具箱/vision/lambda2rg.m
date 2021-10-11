%LAMBDA2RGB RGB chromaticity coordinates
%
% RGB = LAMBDA2RG(LAMBDA) is the rg-chromaticity coordinate (1x2) for 
% illumination at the specific wavelength LAMBDA [m]. If LAMBDA is a
% vector (Nx1), then P (Nx2) is a vector whose elements are the chromaticity
% coordinates at the corresponding elements of LAMBDA.
%
% RGB = LAMBDA2RG(LAMBDA, E) is the rg-chromaticity coordinate (1x2) for an 
% illumination spectrum E (Nx1) defined at corresponding wavelengths
% LAMBDA (Nx1).
%
% References::
%  - Robotics, Vision & Control, Section 10.2,
%    P. Corke, Springer 2011.
%
% See also CMFRGB, LAMBDA2XY.



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
function [r,g] = lambda2rg(lambda, e)
    if nargin == 1,
        RGB = cmfrgb(lambda);
    elseif nargin == 2,
        RGB = cmfrgb(lambda, e);
    end
    cc = tristim2cc(RGB);

    if nargout == 1
        r = cc;
    elseif nargout == 2
        r = cc(:,1);
        g = cc(:,2);
    end
