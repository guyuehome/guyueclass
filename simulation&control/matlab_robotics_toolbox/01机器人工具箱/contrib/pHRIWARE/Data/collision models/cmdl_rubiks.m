%CMDL_RUBIKS Create a Box object of a Rubiks cube
%
% Not an actual CollisionModel, but this script gives an example of some
% more advanced things you can do with objects from the Shape heirachy.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) cmdl_rubiks
%
% See also Box CollisionModel

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

%% Create the box. We use n = 4 for the 3x3 grid, with some other options
rubiks = Box([],0.0285,'n',4,'EdgeColor','k','note','Rubiks Cube demo');

%% Set up the figure nicely, then plot the box to get its handles
figure('NumberTitle','off','Name','Rubiks Cube');
camproj perspective
axis(0.04*[-1 1 -1 1 -1 1]);
daspect([1 1 1]);
light('Position',[1 1 -1]);
light('Position',[-1.5 -1 1]);
light('Position',[-1 -1.5 0.5]);

h = rubiks.plot; % Vector of 6 handles - one for each face

%% Use set to change other properties as desired

set(h(1),'FaceColor',[1 0 0]);
set(h(2),'FaceColor',[0 0 1]);
set(h(3),'FaceColor',[1 1 0]);
set(h(4),'FaceColor',[1 0.5 0]);
set(h(5),'FaceColor',[0 0.5 0]);
set(h(6),'FaceColor',[1 1 1]);
set(h,'LineWidth',5);

%% Create an anitmation sequence and animate using the handles

rubxT = zeros(4,4,360);
for i = 1:360
    rubxT(:,:,i) = troty(i*d2r)*trotz(i*d2r);
end

rubiks.animate(h,rubxT);
clear rubxT i
