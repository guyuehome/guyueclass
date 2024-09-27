%SerialLinked pHRIWARE-linking subclass for SerialLink
%
% Is used to link pHRIWARE functions called by SerialLink objects.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) x = SerialLinked(robot)
%
% Outputs:
%  x : SerialLinked object, pHRIWARE-version copy of SerialLink
%
% Inputs:
%  robot : SerialLink object
%
% See also SerialLink

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.
%
% RTB LIBRARY:
%
% Copyright (C) 1993-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license,
% Modified 19/10/2014 (SerialLinked is a subclass of SerialLink)

classdef SerialLinked < SerialLink
    methods
        function x = SerialLinked(robot)
            x = x@SerialLink(robot);
        end
    end
end

