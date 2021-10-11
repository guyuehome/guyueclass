%RG_ADDTICKS Label spectral locus
%
% RG_ADDTICKS() adds wavelength ticks to the spectral locus.
%
% See also XYCOLOURSPACE.


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

function rg_addticks(lam1, lam2, lamd)

    % well spaced points around the locus
    lambda = [460:10:540];
    lambda = [lambda 560:20:600];

    rgb = cmfrgb(lambda*1e-9);        
    r = rgb(:,1)./sum(rgb')';    
    g = rgb(:,2)./sum(rgb')';    
    hold on
    plot(r,g, 'o')
    hold off

    for i=1:numcols(lambda)
        text(r(i), g(i), sprintf('  %d', lambda(i)));
    end

