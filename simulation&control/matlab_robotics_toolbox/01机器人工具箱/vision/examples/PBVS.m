%PBVS   Implement classical PBVS for point features
%
% A concrete class for simulation of position-based visual servoing (PBVS), a subclass of
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
% plot_z         Plot point depth vs time
% plot_error     Plot feature error vs time
% plot_all       Plot all of the above in separate figures
% char           Convert object to a concise string
% display        Display the object as a string
%
% Example::
%         cam = CentralCamera('default');
%         Tc0 = transl(1,1,-3)*trotz(0.6);
%         TcStar_t = transl(0, 0, 1);
%         pbvs = PBVS(cam, 'T0', Tc0, 'Tf', TcStar_t);
%         pbvs.plot_p
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
% See also VisualServo, IBVS, IBVS_l, IBVS_e.

% IMPLEMENTATION NOTE
%
% 1.  The gain, lambda, is always positive


classdef PBVS < VisualServo

    properties
        lambda          % PBVS gain
        eterm
    end

    methods

        function pbvs = PBVS(cam, varargin)
            %PBVS.PBVS Create PBVS visual servo object
            %
            % PB = PBVS(camera, options)
            %
            % Options::
            % 'niter',N         Maximum number of iterations
            % 'eterm',E         Terminate when norm of feature error < E
            % 'lambda',L        Control gain, positive definite scalar or matrix
            % 'T0',T            The initial pose
            % 'Tf',T            The final relative pose
            % 'P',p             The set of world points (3xN)
            % 'targetsize',S    The target points are the corners of an SxS square
            % 'fps',F           Number of simulation frames per second (default t)
            % 'verbose'         Print out extra information during simulation
            %
            % Notes::
            % - If 'P' is specified it overrides the default square target.
            %
            % See also VisualServo.
            
            % invoke superclass constructor
            pbvs = pbvs@VisualServo(cam, varargin{:});

            % handle arguments
            opt.targetsize = 0.5;       % dimensions of target
            opt.eterm = 0;
            opt.lambda = 0.05;         % control gain

            opt = tb_optparse(opt, pbvs.arglist);

            if isempty(pbvs.Tf)
                pbvs.Tf = transl(0, 0, 1);
                warning('setting Tf to default');
            end

            % copy options to PBVS object
            pbvs.lambda = opt.lambda;
            pbvs.eterm = opt.eterm;
            if isempty(pbvs.niter)
                pbvs.niter = 200;
            end

        end

        function init(vs)
            %PBVS.init Initialize simulation
            %
            % PB.init() initializes the simulation.  Implicitly called by
            % PB.run().
            %
            % See also VisualServo, PBVS.run.
            
            % initialize the vservo variables
            %vs.camera.clf();
            vs.camera.T = vs.T0;    % set camera back to its initial pose
            vs.Tcam = vs.T0;        % initial camera/robot pose
            
            % show the reference location, this is the view we wish to achieve
            % when Tc = T_final
            uv_star = vs.camera.project(vs.P, 'Tcam', inv(vs.Tf));    % create the camera view
            %hold on
            %plot(uv_star(:,1), uv_star(:,2), '*');      % show desired view
            %hold off
            vs.camera.plot(vs.P);    % show initial view
            pause(1)

            % this is the 'external' view of the points and the camera
            plot_sphere(vs.P, 0.05, 'b')
            lighting gouraud
            light
            %cam2 = showcamera(T0);
            vs.camera.plot_camera(vs.P, 'label');
            %camup([0,-1,0]);

            vs.history = [];
        end

        function status = step(vs)
            %PBVS.step Simulate one time step
            %
            % STAT = PB.step() performs one simulation time step of PBVS.  It is
            % called implicitly from the superclass run method.  STAT is
            % one if the termination condition is met, else zero.
            
            status = 0;

            
            % compute the view
            uv = vs.camera.plot(vs.P);

            Tct_est = vs.camera.estpose(vs.P, uv);
            delta =  Tct_est * inv(vs.Tf);
           
            % update the camera pose
            Td = trinterp(delta, vs.lambda);

            vs.Tcam = vs.Tcam * Td;       % apply it to current pose

            % update the camera pose
            vs.camera.T = vs.Tcam;

            % update the history variables
            hist.uv = uv(:);
            vel = tr2delta(Td);
            hist.vel = vel;
            hist.Tcam = vs.Tcam;

            vs.history = [vs.history hist];
            
            if norm(vel) < vs.eterm,
                status = 1;
            end
        end
    end % methods
end % class
