%CMDL_TROPHY Create a CollisionModel object of a grand trophy
%
% Used for demos.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) trophy = cmdl_trophy
%
% See also CollisionModel demo_collisionmodel demo_collisions

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

function trophy = cmdl_trophy()

% Let's being by making a CollisionModel of a precious trophy.
% We will first make the rectangular base, which is 20x20x10 cm
scale = [0.1 0.1 0.05];

% It's centre located in this certain position in our environment
location = [0.4 -0.3 -0.62]';
% Therefore its transform is:
T = [eye(3), location; 0, 0, 0, 1];

% We make the Box object, with some properties to make it look nice
% when plotted, like this:

base = Box(T, scale, 'FaceColor', [0.3 0.2 0.2], 'EdgeColor', 'none');

% We will make the cup of the trophy a Curvilinear object similarly:

scale = [0.2 0.2 0.4];
location = [0.4 -0.3 -0.57]';
T = [eye(3), location; 0, 0, 0, 1];

% The profile of our cup is:
p = @(x) 0.5 - 0.3*cos(4*x);

cup = Curvilinear(T, scale, p, 'FaceColor', [0.6 0.5 0], 'EdgeColor', 'none');

% We will also make the top endcap of the cup not plotted.
cup.faces = [true false];

% We make the CollisionModel object from the two primitives:
trophy = CollisionModel('CollisionModel of a trophy, for demos', base, cup);

end