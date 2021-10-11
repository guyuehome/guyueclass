%MDL_OFFSET3 A minimalistic 3DOF robot arm with shoulder offset
%
% MDL_OFFSET3 is a script that creates the workspace variable off3 which
% describes the kinematic characteristics of a simple arm manipulator with
% a shoulder offset, using standard DH conventions.
%
% Somewhat like a Puma arm without the wrist.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%
% Notes::
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_offset6, mdl_simple6, mdl_puma560.

% MODEL: generic, 3DOF, standard_DH

% Copyright (C) 1993-2015, by Peter I. Corke
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

function r = mdl_offset3()
    
    % robot length values (metres)
    L1 = 1;
    L2 = 1;
    O1 = 0;
    O2 = 0.2;
    O3 = 0;
    
    % and build a serial link manipulator
    
    robot = SerialLink([
        Revolute('alpha', -pi/2, 'a', O1, 'd', 0)
        Revolute('alpha', 0,     'a', L1, 'd', O2)
        Revolute('alpha', pi/2,  'a', L2, 'd', O3)
        ], ...
        'name', 'Offset3');
    
    % place the variables into the global workspace
    if nargout == 1
        r = robot;
    elseif nargout == 0
        assignin('base', 'off3', robot);
        assignin('base', 'qz', [0 0 0 ]); % zero angles, arm up
    end
end
