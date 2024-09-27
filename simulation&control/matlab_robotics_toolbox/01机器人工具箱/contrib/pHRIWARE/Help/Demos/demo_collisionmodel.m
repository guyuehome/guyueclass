%%begin
% Let's being by making a CollisionModel of a precious trophy.
% We will first make the rectangular base, which is 20x20x10 cm
scale = [0.1 0.1 0.05]

% It's centre located in this certain position in our environment
location = [0.4 -0.3 -0.62]'
% Therefore its transform is:
T = [eye(3), location; 0, 0, 0, 1]; T

% We make the Box object, with some properties to make it look nice
% when plotted, like this:

base = Box(T, scale, 'FaceColor', [0.3 0.2 0.2], 'EdgeColor', 'none');

% We will make the cup of the trophy a Curvilinear object similarly:

scale = [0.2 0.2 0.4];
location = [0.4 -0.3 -0.57]';
T = [eye(3), location; 0, 0, 0, 1];

% The profile of our cup is:
p = @(x) 0.5 - 0.3*cos(4*x)

cup = Curvilinear(T, scale, p, 'FaceColor', [0.6 0.5 0], 'EdgeColor', 'none');

% We will also make the top endcap of the cup not plotted.
cup.faces = [true false];

% We make the CollisionModel object from the two primitives:
trophy = CollisionModel(base, cup);

% And we can plot it to see how it looks
figure; light; campos([2 -2 2]); camproj('perspective'); daspect([1 1 1]); axis off
trophy.plot; lighting('gouraud');

% Note that the program sees the cup of the plot as a solid object,
% even though we have plotted is as an open, concave vessel. If the arm
% was to be in the void of the cup but not actually touching, it would
% still be considered colliding.