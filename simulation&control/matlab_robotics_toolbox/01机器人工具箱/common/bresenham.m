%BRESENHAM Generate a line
%
% P = BRESENHAM(X1, Y1, X2, Y2) is a list of integer coordinates (2xN) for 
% points lying on the line segement (X1,Y1) to (X2,Y2).
%
% P = BRESENHAM(P1, P2) as above but P1=[X1,Y1] and P2=[X2,Y2].
%
% Notes::
% - Endpoints must be integer values.
%
% See also ICANVAS.




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

function p = bresenham(x1, y1, x2, y2)

    if nargin == 2
        p1 = x1; p2 = y1;

        x1 = p1(1); y1 = p1(2);
        x2 = p2(1); y2 = p2(2);
    elseif nargin ~= 4
        error('expecting 2 or 4 arguments');
    end

    x = x1;
    if x2 > x1
        xd = x2-x1;
        dx = 1;
    else
        xd = x1-x2;
        dx = -1;
    end

    y = y1;
    if y2 > y1
        yd = y2-y1;
        dy = 1;
    else
        yd = y1-y2;
        dy = -1;
    end

    p = [];

    if xd > yd
      a = 2*yd;
      b = a - xd;
      c = b - xd;

      while 1
        p = [p; x y];
        if all([x-x2 y-y2] == 0)
            break
        end
        if  b < 0
            b = b+a;
            x = x+dx;
        else
            b = b+c;
            x = x+dx; y = y+dy;
        end
      end
    else
      a = 2*xd;
      b = a - yd;
      c = b - yd;

      while 1
        p = [p; x y];
        if all([x-x2 y-y2] == 0)
            break
        end
        if  b < 0
            b = b+a;
            y = y+dy;
        else
            b = b+c;
            x = x+dx; y = y+dy;
        end
      end
    end
end
