%%begin
% In this demo, we are testing for collisions in a situation where a
% person is moving their arm about randomly  while a PUMA 560 robot
% also moves about randomly.

% We will load a saved SerialLink model of a PUMA 560 with STL data:
load('p560_col.mat');
p560

% We will now load the model of our person. First a shoulder transform:
x = 1/sqrt(2);
Tg = [x 0 -x 0; -x 0 -x 0.3; 0 1 0 0.6; 0 0 0 1];
% Now create our HAL object with it, and other options default ([]). We
% then add the CollisionModel objects (head and torso, arm)
hal = HAL(Tg,[],[],[]);
hal.addCM;

% Let's see what we have. The plot3d function is overloaded in HAL:
fig1 = figure;
p560.plot3d(zeros(1,6));
hold on
hal.plot3d(zeros(1,7));

% Let's generate a random set of joint angles. We will ignore joint
% limits for the sake of this demo. We will keep the wrists neutral
q_560 = [2*pi*rand(20,3), zeros(20,3)];
q_man = [2*pi*rand(20,4), zeros(20,3)];

% We need to get the coordinate frames of the upper arm and forearm of
% the person for all 20 time steps. We use the h2fsu method for this:
Th = hal.fkine(q_man); % First make the hand frames
[Tf, ~, Tu] = hal.h2fsu(Th); % ...to forearm and upper arm frames

% Because the wrist is neutral, we do not have to worry about setting a
% method to resolve the swivel angle. We use the MATLAB function cat to
% concatenate Tf and Tu for the collisions method. In the arm
% CollisionModel, the upper arm is the first primitive, so we do:
dyn_T = cat(4, Tu, Tf);
size(dyn_T)

% We check for collisions then by:

c = p560.collisions(q_560, hal.HAT, hal.ARM, dyn_T)

% Let's inspect the first colliding result
% (If there happens to be no collisions, brace yourself for an error -
% sorry! Try running the demo again

i = find(c, 1, 'first');
close(fig1); figure;
p560.plot3d(q_560(i,:));
hold on
hal.plot3d(q_man(i,:));
    