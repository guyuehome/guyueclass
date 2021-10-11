%SPH   Implement spherical IBVS for point features
%
%  results = sph(T)
%  results = sph(T, params)
%
%  Simulate IBVS with for a square target comprising 4 points is placed 
%  in the world XY plane. The camera/robot is initially at pose T and is
%  driven to the orgin.
%
%  Two windows are shown and animated:
%   1. The camera view, showing the desired view (*) and the 
%      current view (o)
%   2. The external view, showing the target points and the camera
%
% The results structure contains time-history information about the image
% plane, camera pose, error, Jacobian condition number, error norm, image
% plane size and desired feature locations.
%
% The params structure can be used to override simulation defaults by
% providing elements, defaults in parentheses:
%
%   target_size    - the side length of the target in world units (0.5)
%   target_center  - center of the target in world coords (0,0,2)
%   niter          - the number of iterations to run the simulation (500)
%   eterm          - a stopping criteria on feature error norm (0)
%   lambda         - gain, can be scalar or diagonal 6x6 matrix (0.01)
%   ci             - camera intrinsic structure (camparam)
%   depth          - depth of points to use for Jacobian, scalar for
%                    all points, of 4-vector.  If null take actual value
%                    from simulation      ([])
%
% SEE ALSO: ibvsplot

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

% IMPLEMENTATION NOTE
%
% 1.  As per task function notation (Chaumette papers) the error is
%     defined as actual-demand, the reverse of normal control system
%     notation.
% 2.  The gain, lambda, is always positive
% 3.  The negative sign is written into the control law

function results = sph(T0, params)

    %% default parameters
    
    % define default target geometry
    vw = 2; vh = 2;       % dimensions of target
    Tct_star = transl(0, 0, -2);    % desired pose of target wrt camera

    % define default IBVS parameters
    niter = 500;            % number of iterations
    eterm = 0;
    lambda = 0.04;         % control gain
    cam = scamera;
    depth = [];
    estim = [];
    
    if nargin == 2,
        % override default target size
        if isfield(params, 'target_size'),
            vw = params.target_size(1);
            vh = params.target_height(1);
        end
        % override default camera desired pose
        if isfield(params, 'target_center'),
            Tct_star = params.Tct_star;
        end
        % override default number of iterations
        if isfield(params, 'niter'),
            niter = params.niter;
        end
        % override default gain (can be scalar or diag 6x6)
        if isfield(params, 'lambda'),
            lambda = params.lambda;
        end
        % override default camera intrinsics
        if isfield(params, 'ci'),
            ci = params.ci;
        end
        % override default point depth
        if isfield(params, 'depth'),
            if length(params.depth) == 1,
                depth = params.depth * ones(4,1);
            else
                depth = params.depth;
            end
        end
        if isfield(params, 'estim'),
            estim = params.estim;
        end
    end

    % define feature points in XY plane
    %  make vertices of a square
    P = mkgrid(2, [vw vh]);
    
    % show the reference location, this is the view we wish to achieve
    % when Tc = Tct_star
    figure(1)
    clf
    P
    Tct_star
    % HACKuv_star = cam.plot(P, inv(Tct_star));    % create the camera view
    uv_star = cam.plot(P, (Tct_star));    % create the camera view
    k=1
    plot(uv_star(2,k), uv_star(1,k), '*');
    hold on
    for k=2:4
        plot(uv_star(2,k), uv_star(1,k), '*');
    end
    %plot(uv_star(:,1), uv_star(:,2), '*');  % show desired view
    %hold off
    %cam.plot(P, T0);    % show initial view
    pause(1)

    % this is the 'external' view of the points and the camera
    figure(2)
    plot3(P(1,:), P(2,:), P(3,:), '*')
    showcam(T0, P)
    grid
    xlabel('x');
    ylabel('y');
    zlabel('z');
    pause
    figure(1)
    %cam2 = showcamera(T0);
    %camup([0,-1,0]);

    %% initialize the vservo variables

    Tcam = T0;                % initial camera/robot pose

    % initialize some history variables, suffix _h, to hold time series data
    % about the IBVS run
    v_h = [];       % velocity demand to robot
    e_h = [];       % feature error
    en_h = [];      % scalar norm of the feature error
    c_h = [];       % Jacobian condition number
    uv_h = [];      % image plane feature coordinates
    Tcam_h = [];
    Z_h = [];
    PP = eye(1,1);
    theta = 0;
    smoothing = 0.95;

    for k=1:niter,
         % set and show the camera pose
        cam.Tcam = Tcam;
        %showcamera(cam2, Tcam);
        figure(2)
        plot3(P(1,:), P(2,:), P(3,:), '*')
        grid on
        xlabel('x');
        ylabel('y');
        zlabel('z');
        showcam(Tcam, P);
        figure(1)
        
        % compute the view
        uv = cam.plot(P);

        e = uv - uv_star;   % as per task function notation
        uv
        uv_star
        e
        if 1
            for i=1:numcols(e)
                % vectorize this
                if e(2,i) > pi
                    e(2,i) = e(2,i) - 2*pi;
                elseif e(2,i) < -pi
                    e(2,i) = 2*pi + e(2,i);
                end
            end
        end
        e
        e = reshape(e, numel(e), 1);
        
        if estim,
            % run the depth estimator
            if k > 1,

                % compute Jacobian for unit depth, r=1
                J = jac(uv, ones(numcols(uv),1));
                Jv = J(:,1:3);  % velocity part, depends on 1/z
                Jw = J(:,4:6);  % rotational part, indepedent of 1/z

                % estimate image plane velocity
                uv_d =  reshape(uv, 1, [])' - reshape(uv_p, 1, [])';
                
                % estimate coefficients for A (1/z) = B
                B = uv_d - Jw*v(4:6);
                A = Jv * v(1:3);

                eta = A\B;          % least squares solution
                
                % first order smoothing
                theta = (1-smoothing) * 1/eta + smoothing * theta;

                pt = transformp(inv(Tcam), P);

                fprintf('depth %.4g, est depth %.4g, rls depth %.4g\n', pt(3,1), 1/eta, theta);
                Z_h(k,:) = [theta pt(3,1)];   % estimated depth, true depth
            end
            uv_p = uv;
        end
        
        
        % compute the Jacobian
        if isempty(depth),
            % exact depth from simulation (not possible in practice)
            pt = transformp(inv(Tcam), P);
            J = jac(uv, colnorm(pt) );
        else
            J = jac(uv, depth );
        end
        J

        % compute the velocity of camera in camera frame

        v = -lambda * pinv(J) * e;
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
        norm(e)

        % update the camera pose
        Td = (eye(4,4) + delta2tr( v ) );    % differential motion
        Td = trnorm(Td);

        Tcam = Tcam * Td;       % apply it to current pose

        % update the history variables
        uv_h(k,:) = reshape(uv([2 1],:), 1, []);
        e_h(k,:) = e';
        en_h(k,:) = norm(e);   
        c_h(k,:) = cond(J);
        v_h(k,:) = v';
        Tcam_h(:,:,k) = Tcam;
        
        if norm(e) < eterm,
            fprintf('completed on error tolerance\n');
            break;
        end
        drawnow
        pause(0.05)
        pause
    end
    fprintf('completed on iteration count\n');
    
    results.uv = uv_h;
    results.uv_labels = {'\phi (rad)', '\theta (rad)'};
    results.c = c_h;
    results.e = e_h;
    results.en = en_h;
    results.v = v_h;
    results.Tcam = Tcam_h;
    %results.limits = [0 cam.nu 0 cam.nv];
    results.limits = [-pi pi 0 pi];
    results.uv_star = uv_star([2 1],:);
