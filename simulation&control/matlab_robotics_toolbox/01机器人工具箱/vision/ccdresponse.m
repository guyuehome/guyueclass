%CCDRESPONSE    CCD spectral response
%
% R = CCDRESPONSE(LAMBDA) is the spectral response of a typical silicon
% imaging sensor at the wavelength LAMBDA [m].  The response is normalized
% in the range 0 to 1.  If LAMBDA is a vector then R is a vector of the
% same length whose elements are the response at the corresponding element
% of LAMBDA.
%
% Notes::
% - Deprecated, use loadspectrum(lambda, 'ccd') instead.
%
% References::
% - An ancient Fairchild data book for a silicon sensor.
% - Robotics, Vision & Control, Section 10.2,
%   P. Corke, Springer 2011.
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
function cc = ccdresponse(lam)
    tab = [300 0
    350 0
    400 5
    500 30
    600 60
    700 85
    800 100
    900 85
    1000 50
    ];
    cc = spline(tab(:,1)*1e-9,tab(:,2),lam)/100;
