%IBVS   Implement classical IBVS for point features
%
% A concrete class for simulation of image-based visual servoing (IBVS), a subclass of
% VisualServo.  Two windows are shown and animated:
%   - The camera view, showing the desired view (*) and the 
%     current view (o)
%   - The external view, showing the target points and the camera
%
% Methods::
% run            Run the simulation, complete results kept in the object
% plot_p         Plot image plane coordinates of points vs time
% plot_vel       Plot camera velocity vs time
% plot_camera    Plot camera pose vs time
% plot_jcond     Plot Jacobian condition vs time 
% plot_z         Plot point depth vs time
% plot_error     Plot feature error vs time
% plot_all       Plot all of the above in separate figures
% char           Convert object to a concise string
% display        Display the object as a string
%
% Example::
%         cam = CentralCamera('default');
%         Tc = trnorm( Tc * delta2tr(v) );
%         Tc0 = transl(1,1,-3)*trotz(0.6);
%         pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], cam.pp');
%         ibvs = IBVS(cam, 'T0', Tc0, 'pstar', pStar)
%         ibvs.run();
%         ibvs.plot_p();
%
% References::
% - Robotics, Vision & Control, Chap 15
%   P. Corke, Springer 2011.
%
% Notes::
% - The history property is a vector of structures each of which is a snapshot at
%   each simulation step of information about the image plane, camera pose, error, 
%   Jacobian condition number, error norm, image plane size and desired feature 
%   locations.
%
% See also VisualServo, PBVS, IBVS_l, IBVS_e.

% IMPLEMENTATION NOTE
%
% 1.  As per task function notation (Chaumette papers) the error is
%     defined as actual-demand, the reverse of normal control system
%     notation.
% 2.  The gain, lambda, is always positive
% 3.  The negative sign is written into the control law

classdef IBVS < VisualServo

    properties
        lambda          % IBVS gain
        eterm
        uv_p            % previous image coordinates

        depth
        depthest
        vel_p
        theta
        smoothing
    end

    methods

        function ibvs = IBVS(cam, varargin)
            %IBVS.IBVS Create IBVS visual servo object
            %
            % IB = IBVS(camera, options)
            %
            % Options::
            % 'niter',N         Maximum number of iterations
            % 'eterm',E         Terminate when norm of feature error < E
            % 'lambda',L        Control gain, positive definite scalar or matrix
            % 'T0',T            The initial pose
            % 'P',p             The set of world points (3xN)
            % 'targetsize',S    The target points are the corners of an SxS square
            % 'pstar',p         The desired image plane coordinates
            % 'depth',D         Assumed depth of points is D (default true depth
            %                   from simulation is assumed)
            % 'depthest'        Run a simple depth estimator
            % 'fps',F           Number of simulation frames per second (default t)
            % 'verbose'         Print out extra information during simulation
            %
            % Notes::
            % - If 'P' is specified it overrides the default square target.
            %
            % See also VisualServo.

            % invoke superclass constructor
            ibvs = ibvs@VisualServo(cam, varargin{:});

            % handle arguments
            opt.eterm = 0.5;
            opt.lambda = 0.08;         % control gain
            opt.depth = [];
            opt.depthest = false;
            
            opt = tb_optparse(opt, ibvs.arglist);

            % copy options to IBVS object
            ibvs.lambda = opt.lambda;
            ibvs.eterm = opt.eterm;
            ibvs.theta = 0;
            ibvs.smoothing = 0.80;
            ibvs.depth = opt.depth;
            ibvs.depthest = opt.depthest;

        end

        function init(vs)
            %IBVS.init Initialize simulation
            %
            % IB.init() initializes the simulation.  Implicitly called by
            % IB.run().
            %
            % See also VisualServo, IBVS.run.

            if ~isempty(vs.pf)
                % final pose is specified in terms of image coords
                vs.uv_star = vs.pf;
            else
                if ~isempty(vs.Tf)
                    vs.Tf = transl(0, 0, 1);
                    warning('setting Tf to default');
                end
                % final pose is specified in terms of a camera-target pose
                %   convert to image coords
                vs.uv_star = vs.camera.project(vs.P, 'Tcam', inv(vs.Tf));
            end

            % initialize the vservo variables
            vs.camera.T = vs.T0;    % set camera back to its initial pose
            vs.Tcam = vs.T0;                % initial camera/robot pose
            
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

            vs.camera.plot(vs.P);    % show initial view

            % this is the 'external' view of the points and the camera
            plot_sphere(vs.P, 0.05, 'b')
            lighting gouraud
            light
            %cam2 = showcamera(T0);
            vs.camera.plot_camera(vs.P, 'label');
            %camup([0,-1,0]);

            vs.vel_p = [];
            vs.uv_p = [];
            vs.history = [];
        end

        function status = step(vs)
            %IBVS.step Simulate one time step
            %
            % STAT = IB.step() performs one simulation time step of IBVS.  It is
            % called implicitly from the superclass run method.  STAT is
            % one if the termination condition is met, else zero.
            %
            % See also VisualServo, IBVS.run.
            
            status = 0;
            Zest = [];
            
            % compute the view
            uv = vs.camera.plot(vs.P);

            % optionally estimate depth
            if vs.depthest
                % run the depth estimator
                [Zest,Ztrue] = vs.depth_estimator(uv);
                if vs.verbose
                    Zest
                    Ztrue
                end
                vs.depth = Zest;
                hist.Ztrue = Ztrue(:);
                hist.Zest = Zest(:);
            end

            % compute image plane error as a column
            e = uv - vs.uv_star;   % feature error
            e = e(:);
        
            
            % compute the Jacobian
            if isempty(vs.depth)
                % exact depth from simulation (not possible in practice)
                pt = homtrans(inv(vs.Tcam), vs.P);
                J = vs.camera.visjac_p(uv, pt(3,:) );
            elseif ~isempty(Zest)
                J = vs.camera.visjac_p(uv, Zest);
            else
                J = vs.camera.visjac_p(uv, vs.depth );
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

            % update the camera pose
            Td = trnorm(delta2tr(v));    % differential motion

            vs.Tcam = vs.Tcam * Td;       % apply it to current pose
            vs.Tcam = trnorm(vs.Tcam);

            % update the camera pose
            vs.camera.T = vs.Tcam;

            % update the history variables
            hist.uv = uv(:);
            vel = tr2delta(Td);
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e);
            hist.jcond = cond(J);
            hist.Tcam = vs.Tcam;

            vs.history = [vs.history hist];

            vs.vel_p = vel;
            vs.uv_p = uv;

            if norm(e) < vs.eterm,
                status = 1;
                return
            end
        end

        function [Zest,Ztrue] = depth_estimator(vs, uv)
            %IBVS.depth_estimator Estimate point depth
            %
            % [ZE,ZT] = IB.depth_estimator(UV) are the estimated and true world 
            % point depth based on current feature coordinates UV (2xN).
            
            if isempty(vs.uv_p)
                Zest = [];
                Ztrue = [];
                return;
            end

            % compute Jacobian for unit depth, z=1
            J = vs.camera.visjac_p(uv, 1);
            Jv = J(:,1:3);  % velocity part, depends on 1/z
            Jw = J(:,4:6);  % rotational part, indepedent of 1/z

            % estimate image plane velocity
            uv_d =  uv(:) - vs.uv_p(:);
            
            % estimate coefficients for A (1/z) = B
            B = uv_d - Jw*vs.vel_p(4:6);
            A = Jv * vs.vel_p(1:3);

            AA = zeros(numcols(uv), numcols(uv)/2);
            for i=1:numcols(uv)
                AA(i*2-1:i*2,i) = A(i*2-1:i*2);
            end
            eta = AA\B;          % least squares solution

            eta2 = A(1:2) \ B(1:2);

            % first order smoothing
            vs.theta = (1-vs.smoothing) * 1./eta' + vs.smoothing * vs.theta;
            Zest = vs.theta;

            % true depth
            P_CT = homtrans(inv(vs.Tcam), vs.P);
            Ztrue = P_CT(3,:);

            if vs.verbose
                fprintf('depth %.4g, est depth %.4g, rls depth %.4g\n', ...
                    Ztrue, 1/eta, Zest);
            end
        end
    end % methods
end % class
