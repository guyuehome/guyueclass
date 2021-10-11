%MTOOLS	add simple/useful tools to all windows in figure
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
function mtools
	global DDXFILENAME

	h = uicontextmenu;
	uimenu(h, 'Label', 'X zoom', 'CallBack', 'xaxis');
	uimenu(h, 'Label', '-->', 'CallBack', 'xscroll(0.5)');
	uimenu(h, 'Label', '<--', 'CallBack', 'xscroll(-0.5)');
	uimenu(h, 'Label', 'CrossHairs', 'CallBack', 'crosshair');
	uimenu(h, 'Label', 'X UNzoom', 'CallBack', 'unzoom');
	uimenu(h, 'Label', 'Pick delta', 'CallBack', 'fprintf(''%f %f\n'', diff(ginput(2)))');
	uimenu(h, 'Label', 'Line fit', 'CallBack', 'ilinefit');
	uimenu(h, 'Label', 'Show points', 'CallBack', 'showpoints(gca)');
	uimenu(h, 'Label', 'Apply X zoom to all', 'CallBack', 'xaxisall');
	for c=get(gcf, 'Children')',
		set(c, 'UIContextMenu', h);
		l = get(c, 'Children');
	end

	
  	axes('pos', [0 0 1 0.05], 'visible', 'off')
    if 0
        if ~isempty(DDXFILENAME),
            s = sprintf('[%s]  %s', DDXFILENAME, date);
        else
            s = sprintf('%s', date);
        end
        text(0.95, 0.1, s, 'horizontalalign', 'right', 'verticalalign', 'baseli')      
    end
