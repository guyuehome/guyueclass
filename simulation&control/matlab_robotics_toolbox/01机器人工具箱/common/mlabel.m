%MLABEL	labels for mplot style graph
%
%	mlabel({lab1 lab2 lab3})

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

function mlabel(lab, varargin)

	% find all child axes (subplots)
	h = findobj(gcf, 'Type', 'axes');

	for i=1:length(h),

		if strcmp( get(h(i), 'visible'), 'on'),
			axes(h(i))
			% get subplot number from user data (I don't know who
			% sets this but its very useful)
			sp = get(h(i), 'UserData');
			if sp == 1,
				topplot = sp;
			end
			ylabel(lab{sp}, varargin{:});
		end
	end

    if 0
        if nargin > 1,
            axes(h(topplot));	% top plot
            title(tit);
        end
    end
