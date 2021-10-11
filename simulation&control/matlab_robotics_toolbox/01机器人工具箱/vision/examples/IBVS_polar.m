%IBVS   Implement classical IBVS for point features
%
%  results = ibvs(T)
%  results = ibvs(T, params)
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
%   target_center  - center of the target in world coords (0,0,3)
%   niter          - the number of iterations to run the simulation (500)
%   eterm          - a stopping criteria on feature error norm (0)
%   lambda         - gain, can be scalar or diagonal 6x6 matrix (0.01)
%   ci             - camera intrinsic structure (camparam)
%   depth          - depth of points to use for Jacobian, scalar for
%                    all points, of 4-vector.  If null take actual value
%                    from simulation      ([])
%
% SEE ALSO: ibvsplot

% IMPLEMENTATION NOTE
%
% 1.  As per task function notation (Chaumette papers) the error is
%     defined as actual-demand, the reverse of normal control system
%     notation.
% 2.  The gain, lambda, is always positive
% 3.  The negative sign is written into the control law

classdef IBVS_polar < VisualServo

    properties
        rt          % r, theta
        rt_star

        lambda          % IBVS gain
        eterm

        depth

        h_rt
    end

    methods

        function ibvs = IBVS_polar(cam, varargin)

            % invoke superclass constructor
            ibvs = ibvs@VisualServo(cam, varargin{:});

            % handle arguments
            opt.eterm = 0.001;
            opt.lambda = 0.02;         % control gain
            opt.depth = [];
            
            opt = tb_optparse(opt, ibvs.arglist);

            % copy options to IBVS object
            ibvs.lambda = opt.lambda;
            ibvs.eterm = opt.eterm;
            ibvs.depth = opt.depth;
        end

        function init(vs)
            vs.h_rt
            ishandle(vs.h_rt)
            if 0 % isempty(vs.h_rt) || ~ishandle(vs.h_rt)
                fprintf('create rt axes\n');
                vs.h_rt = axes;
                set(vs.h_rt, 'XLimMode', 'manual');
                set(vs.h_rt, 'YLimMode', 'manual');
                set(vs.h_rt, 'NextPlot', 'replacechildren');
                axis([-pi pi 0 sqrt(2)])
                %axis([-pi pi 0 norm(vs.camera.npix-vs.camera.pp)])
                xlabel('\theta (rad)');
                ylabel('r (pix)');
                title('polar coordinate feature space');
                grid
            end
            %axes(vs.h_rt)
            %cla

            if isempty(vs.Tf)
                vs.Tf = transl(0, 0, -1);
                warning('setting Tf to default');
            end
            %% initialize the vservo variables
            vs.camera.T = vs.T0;    % set camera back to its initial pose
            vs.Tcam = vs.T0;                % initial camera/robot pose

            % final pose is specified in terms of a camera-target pose
            %   convert to image coords
            vs.rt_star = vs.project(vs.P, vs.Tf);

            
            % show the reference location, this is the view we wish to achieve
            % when Tc = Tct_star
            if 0
            vs.camera.clf()
            vs.camera.plot(vs.uv_star, '*'); % create the camera view
            vs.camera.hold(true);
            vs.camera.plot(vs.P, 'Tcam', vs.T0, 'o'); % create the camera view
            pause(2)
            vs.camera.hold(false);
            vs.camera.clf();
            end

            %vs.camera.plot(vs.P);    % show initial view

            % this is the 'external' view of the points and the camera
            %plot_sphere(vs.P, 0.05, 'b')
            %cam2 = showcamera(T0);
            vs.camera.visualize(vs.P, 'label');
            %camup([0,-1,0]);


            vs.history = [];
        end

        function status = step(vs)
            status = 0;
            Zest = [];
            
            % compute the view
            uv = vs.camera.project(vs.P, 'Tcam', vs.Tcam);
            rt = vs.project(vs.P);

            % compute image plane error as a column
            e = rt - vs.rt_star;   % feature error
            e(2,:) = angdiff(e(2,:));
            e = e(:);
            
        
            % compute the Jacobian
            if isempty(vs.depth)
                % exact depth from simulation (not possible in practice)
                P_C = homtrans(inv(vs.Tcam), vs.P);
                P_C(3,:)
                J = vs.camera.visjac_p_polar(rt, P_C(3,:) );
            else
                J = vs.camera.visjac_p_polar(rt, vs.depth );
            end

            % compute the velocity of camera in camera frame
            try
                v = -vs.lambda * pinv(J) * e;
            catch
                status = -1;
                return
            end

            if vs.verbose
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            end

            if norm(v) > 0.01
                v = unit(v) * 0.01;
            end
            
            % update the camera pose
            Td = trnorm(delta2tr(v));    % differential motion

            vs.Tcam = vs.Tcam * Td;       % apply it to current pose
            vs.Tcam = trnorm(vs.Tcam);

            % update the camera pose
            vs.camera.T = vs.Tcam;

            % update the history variables
            hist.uv = uv(:);
            hist.rt = rt(:);
            vel = tr2diff(Td);
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e);
            hist.jcond = cond(J);
            hist.Tcam = vs.Tcam;

            vs.history = [vs.history hist];

            if norm(e) < vs.eterm,
                status = 1;
                return
            end
        end

        function rt = project(vs, P, T)

            if nargin < 3
                99
                T = vs.Tcam;
            end
            T
            p = vs.camera.plot(P, 'Tcam', T);
            %p = homtrans( inv(vs.camera.K), p);

            pp = vs.camera.pp;
            u = p(1,:) - pp(1); v = p(2,:) - pp(2);
            rt = [sqrt(u.^2+v.^2)/vs.camera.npix(1); atan2(v,u)];

            %line(rt(:,2), rt(:,1), 'Marker', 'o', 'MarkerFaceColor', 'k', 'Parent', vs.h_rt)

            % plot points on rt plane
        end

        function plot_features(vs)
            clf
            hold on
            % r-theta plane trajectory
            rt = [vs.history.rt]';
            % result is a vector with row per time step, each row is r1, th1, r2, th2...
            for i=1:numcols(rt)/2
                p = rt(:,i*2-1:i*2);    % get data for i'th point
                plot(p(:,2), p(:,1))

                plot(p(1,2), p(1,1), 'o');
                plot(vs.rt_star(2,i), vs.rt_star(1,1), '*');
            end
            axis([-pi pi 0 sqrt(2)]);
            grid
            xlabel('\phi (rad)');
            ylabel('r (pixels)');
            hold off
        end

    end % methods
end % class
