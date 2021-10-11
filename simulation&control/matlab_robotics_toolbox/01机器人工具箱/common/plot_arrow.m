%PLOT_ARROW Draw an arrow
%
% PLOT_ARROW(P, OPTIONS) draws an arrow from P1 to P2 where P=[P1; P2].
%
% Options::
% All options are passed through to arrow3.  Pass in a single character
% MATLAB colorspec (eg. 'r') to set the color.
%
% See also ARROW3.

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
function plot_arrow(p, varargin)
    mstart = p(1:end-1,:);
    mend = p(2:end,:);
    %mstart = p;
    %mend = [p(2:end,:); p(1,:)];

    arrow3(mstart, mend, varargin{:});
