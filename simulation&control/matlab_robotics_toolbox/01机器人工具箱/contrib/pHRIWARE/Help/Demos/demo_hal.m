%%begin
% We will begin by constructing a HAL object with default properties

hal = HAL()

% Let's plot it in the zero angle configuration
qz = zeros(1,7);
hal.plot(qz); campos([6.5 6 5]);
% Note with a neutral wrist, the x-axis is in the direction of the
% forearm, and the z-axis is the elbow axis.

% We can see already that as a subclass, HAL inherits lots of 
% functionality SerialLink objects. Forward kinematics is the same too
Th = hal.fkine(qz)

% Methods that are overloaded are ikine and islimit. Check out the help
% or doc files for these methods.
%
% Extending these is the method reachable, which combines the two
% inverse kinematic solutions and checks for joint limits. Poses that
% are unreachable are ouput as a NaN row.
[q1, q2] = hal.ikine(Th)
q = hal.reachable(q1, q2)

% You can use the method h2fsu to convert hand frames into forearm,
% swivel and upper arm frames (hence h2fsu). You can also use the hand
% point only - in which case the swivel angle must be explictly specified,
% or h2fsu uses the hand-to-goal method.
[Tf, Ts, Tu] = hal.h2fsu(Th(1:3,4))

% h2fsu in this case has used the hand-to-goal method of resolving the
% swivel angle. The elbow axis is made perpendicular to the goal vector
hal.goal
dot(hal.goal, Tf(1:3,3))
% The dot product is near-zero which says they are perpendicular.

% Let's see what it looks like. We will use plot3d this time, but first
% we must add the collision models to our HAL object.
[q1, q2] = hal.ikine(Tf)
q = hal.reachable(q1, q2)
hal.addCM;
hal.plot3d(q); campos([6.5 6 5]);

