%YAYIS	set Y-axis scaling
%
% YAXIS(MAX) set y-axis scaling from 0 to MAX.
%
% YAXIS(MIN, MAX) set y-axis scaling from MIN to MAX.
%
% YAXIS([MIN MAX]) as above.
%
% YAXIS restore automatic scaling for y-axis.
%
% See also YAXIS.

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

function yaxis(a1, a2)
	if nargin == 0,
		set(gca, 'YLimMode', 'auto')
		return
	elseif nargin == 1,
		if length(a1) == 1,
			mn = 0;
			mx = a1;
		elseif length(a1) == 2,
			mn = a1(1);
			mx = a1(2);
		end
	elseif nargin == 2,
		mn = a1;
		mx = a2;
	end

	set(gca, 'YLimMode', 'manual', 'YLim', [mn mx])
