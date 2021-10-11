%SFM1 Example of structure from motion estimation
close all

nsteps = 50;
noise = 0;
cubeside = 0.6;
pathradius = 3;

% this is the point whose data we plot later
iwp = 8;

randinit
camera = CentralCamera('name', 'sfm', 'default', 'noise', noise);

% create the shape that we will view
P = mkcube(cubeside, 'facepoint');
% 8 pts is too few to get a good F estimate, maybe add cube face
% add points at the centre of each face

% move smoothly along a path 
[theta,theta_d] = tpoly(0, 2*pi, nsteps);

% move in a circle around the object in the xy-plane
%   the camera's z-axis points toward the centre, y-axis downward
T = [];
for th=theta'
    TT = trotz(-th) * transl(0, -pathradius, 0) * trotx(-pi/2);
    T = cat(3, T, TT);
end

% choose a point and assume that it is on the ground plane
%  at a height of z=Zknown.  In the camera frame, y-axis down, this corresponds
%  to y=Yknown.

k = 1;
Pc = homtrans( inv(T(:,:,1)), P(:,k) );
Yknown = Pc(2);


% project the points to the image plane for all camera poses
p = camera.project(P, 'Tcam', T);

res = [];
rp = [];
T_est(:,:,1) = eye(4,4);
T_true(:,:,1) = eye(4,4);
resid = [];
Adet = [];
sigma = [];
worldpoint = [];
for i=2:nsteps
    fprintf('------------step %d\n', i);

    % estimate camera motion
    [F,resid(i-1)] = fmatrix(p(:,:,i-1), p(:,:,i));
    E = camera.E(F);
    relpose = camera.invE(E, [0 0 5]');

    rawscale(i) = norm(transl(relpose));

    dT_true = inv(T(:,:,i-1))*T(:,:,i)

    % for the point known to be on the ground plane, find where the
    % ray corresponding to that point intersects the ground plane
    r1 = camera.ray(p(:,k,i-1));
    Pground = r1.intersect([0 1 0 -Yknown])

    % relpose is the transform from camera 1 to camera 2
    %  we want (R,t) components of the inverses
    [R,t] = tr2rt(inv(relpose));
    
    % the camera matrix is (M,p4)
    C = camera.C;
    M = C(1:3,1:3); p4 = C(:,4);

    % the coefficients of our expression
    a = M*t;
    b= M*R*Pground+p4;
    ph = e2h(p(:,k,i)); % projection of world point in homog form

    % solve the linear equation for scale factors
    %  phi = (sigma, alpha, beta)
    %
    % where:
    %   sigma is the scale factor for translation t
    %   lambda is the projection scale factor
    %   alpha  is error in the direction a x p
    % 
    % The simple solution
    %   A = [-a ph cross(a,ph)];
    %   phi = inv(A) * b
    % is poorly conditioned so we form A from unit vectors and
    % rescale afterward
    A = [-unit(a) unit(ph) unit(cross(a,ph))];
    phi = inv(A) * b
    phi = diag([1/norm(a) 1/norm(ph) 1/norm(cross(a,ph))]) * phi;

    Adet(i-1) = det(A);

    disp('estimated scaled transformation');
    dT_est = inv([R phi(1)*t; 0 0 0 1])

    disp('reprojection error (pixels) in camera 2 using estimated pose')
    camera.move(relpose).project(Pground) - p(:,k,i)

    sigma(i-1,1) = norm(transl(dT_est));
    sigma(i-1,2) = norm(transl(dT_true));
    T_est(:,:,i) = T_est(:,:,i-1) * dT_est;
    T_true(:,:,i) = T_true(:,:,i-1) * dT_true;

    r1 = camera.ray(p(:,iwp,i-1));
    r2 = camera.move(dT_est).ray(p(:,iwp,i));
    worldpoint(:,i-1) = homtrans( T_est(:,:,i-1), r1.intersect(r2) );

    if i == 20
        %break
    end
    if i > 0
        pause
    end

end

% plot some results
figure
plot(resid');
title('F matrix residual');
xlabel('step');
ylabel('residual')

figure
plot(Adet);
title('determinant of A')
xlabel('step');
ylabel('det(A)')

figure
axis([-4 4 -4 4 -1 7]);
hold on
plotp(transl(T_est)', 'x');  % estimated
plotp(transl(T_true)', 'ro'); % true
view(-48,34)
legend('estimated', 'true')
grid

figure
plot(sigma(:,1), 'x')   % estimated
hold on
plot(sigma(:,2), 'r-')   % true
title('est |t|');
grid
xlabel('step');
ylabel('translational scale')
legend('estimated', 'true')

figure
plotp(worldpoint, '.');
grid
hold on
plotp( homtrans(inv(T(:,:,1)), P(:,iwp)), 'rd', 'MarkerFaceColor', 'r');
zm=mean(worldpoint')';
plotp( zm, 'go', 'MarkerFaceColor', 'g');

figure
plot(rawscale);
hold on
plot(sigma(:,2), 'r-')   % true
title('est |t|');
xaxis(2,50);
xt
ylabel('translation magnitude |t|')
legend('estimated', 'true');

grid
