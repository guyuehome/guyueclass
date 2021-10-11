

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
function sphere_paint(sph, varargin)

    % if only a hemisphere pad the other hemisphere with grey
    if nargin > 1
        if strcmp(varargin(1), 'north')
            sph = [sph; 0.3*ones(size(sph))];
        elseif strcmp(varargin(1), 'south')
            sph = [0.3*ones(size(sph)); sph];
        end
    end

    % flip it because the 'top' of the Matlab sphere function is
    % the south pole...
    sph = flipud(sph);

    % create the sphere and get handle
    sphere
    h=findobj('Type', 'surface');

    set(h,'cdata', sph, 'facecolor', 'texture');
    colormap(gray)
