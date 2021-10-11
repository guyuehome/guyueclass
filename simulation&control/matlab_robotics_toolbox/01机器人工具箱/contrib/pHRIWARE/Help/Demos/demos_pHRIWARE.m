%DEMOS_PHRIWARE shows some demos of the pHRIWARE software package
% 
% Opens a menu to choose some demonstrations, which give step-by-step
% explanations in the Command Window of using certain functionalities
% of pHRIWARE. Based on the demo format of RTB by Peter Corke.
% 
% Copyright (C) Bryan Moutrie, 2013-014
% Licensed under the GNU General Public License, see file for statement
% 
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% See also pHRIWARE

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.
%
% RTB LIBRARY:
%
% Copyright (C) 1993-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license,
% Modified 24/7/2014 (code adapted from rtbdemo.m, uses runscript.m)

demos = {
    '*HAL basics', 'hal';
    '*Using HAL', 'hal2';
    'Load HAT collision model', 'hat';
    'Create a CollisionModel', 'collisionmodel';
    '*collisions basics', 'collisions';
    '*collisions with dynamic objects', 'collisions2';
    'Exit', '';
    };

while true
    selection = menu('*Requires Robotics Toolbox', demos{:,1});
    
    if strcmp(demos{selection,1}, 'Exit')
        % quit now
        delete( get(0, 'Children') );
        break;
    else
        % run the appropriate script
        script = demos{selection,2};
        runscript(['demo_', script])
    end
end