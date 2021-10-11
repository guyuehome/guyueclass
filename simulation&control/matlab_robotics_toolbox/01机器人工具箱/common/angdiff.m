%ANGDIFF Difference of two angles
%
% D = ANGDIFF(TH1, TH2) returns the difference between angles TH1 and TH2 on
% the circle.  The result is in the interval [-pi pi).  If TH1 is a column 
% vector, and TH2 a scalar then return a column vector where TH2 is modulo 
% subtracted from the corresponding elements of TH1.
%
% D = ANGDIFF(TH) returns the equivalent angle to TH in the interval [-pi pi).
%

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

function d = angdiff(th1, th2)

    if nargin < 2
% THIS IS A BAD IDEA, WHERE IS IT USED?
%         if length(th1) > 1
%             d = th1(1) - th1(2);
%         else
%             d = th1;
%         end
        d = th1;
    else
        d = th1 - th2;
    end

    
    d = mod(d+pi, 2*pi) - pi;

% Simplistic version of the code, easy to see what it does, but slow...
%
% for very negative angles keep adding 2pi
%     while true
%         k = find(d < -pi);
%         if isempty(k)
%             break;
%         end
%         d(k) = d(k) + 2*pi;
%     end
% 
%     % for very positive angles keep subtracting 2pi
%     while true
%         k = find(d > pi);
%         if isempty(k)
%             break;
%         end
%         d(k) = d(k) - 2*pi;
%     end
