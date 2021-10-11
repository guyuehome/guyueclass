%DIFF2 First-order difference
%
% D = DIFF2(V) is the first-order difference (1xN) of the series data in 
% vector V (1xN) and the first element is zero.
%
% D = DIFF2(A) is the first-order difference (MxN) of the series data in 
% each row of the matrix A (MxN) and the first element in each row is zero.
%
% Notes::
% - Unlike the builtin function DIFF, the result of DIFF2 has the same
%   number of columns as the input.
%
% See also DIFF.

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
function d = diff2(v)
	[r,c] =size(v);

	d = [zeros(1,c); diff(v)];
