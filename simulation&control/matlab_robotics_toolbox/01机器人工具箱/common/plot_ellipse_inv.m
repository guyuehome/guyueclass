%PLOT_ELLIPSE_INV Draw an ellipse or ellipsoid
%
% PLOT_ELLIPSE_INV(A, OPTIONS) draws an ellipse defined by X'.inv(A).X = 0 on the
% current plot, centred at the origin.
%
% PLOT_ELLIPSE_INV(A, C, OPTIONS) as above but centred at C=[X,Y].  If
% C=[X,Y,Z] the ellipse is parallel to the XY plane but at height Z.
%
% H = PLOT_ELLIPSE_INV(A, C, OPTIONS) as above but return graphic handle.
%
% Options::
% 'edgecolor'   the color of the circle's edge, Matlab color spec
% 'fillcolor'   the color of the circle's interior, Matlab color spec
% 'alpha'       transparency of the filled circle: 0=transparent, 1=solid
% 'alter',H     alter existing circles with handle H
%
% Notes::
% - For the case where the inverse of ellipse parameters are known, perhaps
%   an inverse covariance matrix.
% - If A (2x2) draw an ellipse, else if A(3x3) draw an ellipsoid.
% - The ellipse is added to the current plot.
%
% See also PLOT_ELLIPSE, PLOT_CIRCLE, PLOT_BOX, PLOT_POLY.

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
% See also PLOT_ELLIPSE, PLOT_CIRCLE, PLOT_BOX, PLOT_POLY.


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

function h = plot_ellipse_inv(A, xc, varargin)

    if nargin == 1
        h = plot_ellipse(inv(A));
    elseif nargin == 2
        h = plot_ellipse(inv(A), xc);
    else
        h = plot_ellipse(inv(A), xc, varargin{:});
    end
