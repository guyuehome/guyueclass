%%begin
% We will begin with the trophy from the CollisionModel demo, which we
% do not want to hit.
trophy = cmdl_trophy

% We will load a model of the SerialLink object with STL data included.
load('p560_col.mat');
p560

% The two are precariously close!
fig1 = figure;
qz = zeros(1,6);
p560.plot3d(qz);
hold on
trophy.plot;

% The point data from the STL data is not dense. We can only check for
% collisions where there are points. You can rectify this in a solid
% model program, such as SolidWorks, by adding a grid of split lines on
% the relevant surfaces.

% We can check for collisions
c = p560.collisions(qz, trophy)

% The false/zero value indicates the two are not colliding. If the 2nd
% joint turns a little to move the red link lower,
q = [0 -0.1 0 0 0 0]

c = p560.collisions(q, trophy)

% There is now a collision. Let's verify this
close(fig1); figure;
p560.plot3d(q);
hold on
trophy.plot;

% The robot does indeed collide with the trophy. It is important to
% note that the program sees the cup of the plot as a solid object,
% even though we have plotted is as an open, concave vessel. If the arm
% was to be in the void of the cup but not actually touching, it would
% still be considered colliding.