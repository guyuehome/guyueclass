%%begin
% Lets find the reach of an average person, using the HAL's AP property
% AP (arm properties) is a cell contating:
% base (Tg), upper arm length (gre), forearm length (erw) and goal
hal = HAL();
hal.addCM;
hal.AP
reach = hal.AP{2} + hal.AP{3}

% We can specify our workspace with x, y, z vectors
x = linspace(-reach, reach, 25);
y = x;
z = x;

% And generate a set of hand frames
Tf = hal.h2fsu(x, y, z, d2r*40);

% Let's have a look at the size of Tf
size(Tf)
% Tf is a 6-D array, the first two dimensions representing a single 4x4
% transformation matrix, and then dimensions for the x, y, z and swivel
% angle variables. In this method, the swivel angle has been
% discretised by starting at the median humanly reachable swivel angle,
% and then taking steps +/- 40 degrees from this point. Look-up tables
% are used to do this.

% Points in the Tf array which are unreachable due to their location or
% due to the swivel angle have their place held with NaNs:
Tf(:,:,1,1,1,1)

% To view the workspace of the human arm,
% We select the middle index of the swivel angle dimension as this
% corresponds to the median swivel angle. Therefore all points which
% are physically reachable will be non-NaN. We will still remove the
% points in space which are not reachable and therefore are NaN
pf = squeeze(Tf(1:3,4,:,:,:,4));
pf(isnan(pf)) = [];
pf = reshape(pf, 3, []);

figure;
hal.plot3d(zeros(1,7)); hold on
scatter3(pf(1,:), pf(2,:), pf(3,:));