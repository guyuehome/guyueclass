%%begin
% We can create a CollisionModel object of the human head and torso by
% running cmdl_hat. This takes a couple of seconds.
hat = cmdl_hat;

% The time is spent performing symbolic arithmetic to build the check
% functions. We can plot the collision model to see how it looks:
hat.plot;

% Let's make it a bit better
campos([1 1 0.5]);
light; lighting gouraud

% We can specify a different shoulder frame of the shoulder for the HAT
% model. The default value is a rotation of -90 deg about the x-axis.
% The indentity matrix will therefore rotate the body to be lying down,
% facing down.
hat2 = cmdl_hat(eye(4));

% Options passed to plot operate on all of the Collision Model's
% primitives
hold on
hat2.plot('FaceAlpha', 0.5);