%LOADSPECTRUM Load spectrum data
%
% S = LOADSPECTRUM(LAMBDA, FILENAME) is spectral data (NxD) from file FILENAME 
% interpolated to wavelengths [metres] specified in LAMBDA (Nx1).  The
% spectral data can be scalar (D=1) or vector (D>1) valued.
%
% [S,LAMBDA] = LOADSPECTRUM(LAMBDA, FILENAME) as above but also returns the 
% passed wavelength LAMBDA.
%
% Notes::
% - The file is assumed to have its first column as wavelength in metres, the 
%   remainding columns are linearly interpolated and returned as columns of S.
% - The files are kept in the private folder inside the MVTB folder.
%
% References::
%  - Robotics, Vision & Control, Section 14.3,
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

function [s,lam] = loadspectrum(lambda, filename)

    lambda = lambda(:);
    tab = load(filename);

    s = interp1(tab(:,1), tab(:,2:end), lambda, 'linear', 0);

    if nargout == 2
        lam = lambda;
    end
