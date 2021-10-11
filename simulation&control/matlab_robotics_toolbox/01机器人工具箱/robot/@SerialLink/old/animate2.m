%SerialLink.animate   Update a robot animation
%
% R.animate(q) updates an existing animation for the robot R.  This will have
% been created using R.plot().
%
% Updates graphical instances of this robot in all figures.
%
% Notes::
% - Not a general purpose method, used for Simulink robot animation.
%
% See also SerialLink.plot.

function animate(robot, q, handles)

    if nargin < 3
        handles = findobj('Tag', robot.name);
    end

    for handle=handles'     % for each graphical robot instance
        h = get(handle, 'UserData');
        animate2( h, q);
    end


function animate2(h, q)

    robot = h.robot;
    n = robot.n;
    L = robot.links;

    mag = h.mag;

    % compute the link transforms, and record the origin of each frame
    % for the animation.

    
    % points on the multiline, skeleton of the arm
    x = zeros(1,n);
    y = zeros(1,n);
    z = zeros(1,n);
    
    % points on the multiline, skeleton of the shadow
    xs = zeros(1,n);
    ys = zeros(1,n);
    zs = zeros(1,n);

    % add first point to the link/shadow line, the base
    
    t = robot.base;
    
    x(1) = t(1,4);
    y(1) = t(2,4);
    z(1) = t(3,4);
    xs(1) = t(1,4);
    ys(1) = t(2,4);
    zs(1) = h.zmin;
    
    Tn = t;  % robot base

    if robot.mdh == 0
        % standard DH parameterization
        
        % Tn(:,:,j) is pose of joint j
        for j=1:n
            Tn(:,:,j) = t;
            
            t = t * L(j).A(q(j));
            
            % add point to arm and shadow skeleton
            %   first point is the base
            x(j+1) = t(1,4);
            y(j+1) = t(2,4);
            z(j+1) = t(3,4);
            xs(j+1) = t(1,4);
            ys(j+1) = t(2,4);
            zs(j+1) = h.zmin;
        end
        t = t *robot.tool;
    else
        % modified DH parameterization
        for j=1:n
                        x(j+1) = t(1,4);
            y(j+1) = t(2,4);
            z(j+1) = t(3,4);
            xs(j+1) = t(1,4);
            ys(j+1) = t(2,4);
            zs(j+1) = h.zmin;
            
            t = t * L(j).A(q(j));
                        Tn(:,:,j) = t;

            
            % add point to arm and shadow skeleton
            %   first point is the base

        end
        t = t *robot.tool;

    end

    %
    % draw the robot stick figure and the shadow
    %
    set(h.links,'xdata', x, 'ydata', y, 'zdata', z);
    if isfield(h, 'shadow')
        set(h.shadow,'xdata', xs, 'ydata', ys, 'zdata', zs);
    end
    

    %
    % display the joints as cylinders with rotation axes
    %
    if isfield(h, 'joint')
        xyz_line = [0 0; 0 0; -2*mag 2*mag; 1 1];

        for j=1:n
            % get coordinate data from the cylinder
            xyz = get(h.joint(j), 'UserData');
            
            % transform to current joint frame
            xyz = Tn(:,:,j) * xyz;
            
            % convert to 1-vector
            ncols = numcols(xyz)/2;
            xc = reshape(xyz(1,:), 2, ncols);
            yc = reshape(xyz(2,:), 2, ncols);
            zc = reshape(xyz(3,:), 2, ncols);
            
            % and update the coordinates of the graphical model
            set(h.joint(j), 'Xdata', xc, 'Ydata', yc, ...
                'Zdata', zc);

            if isfield(h, 'jointaxis')
                % optionally draw the joint axis
                
                % transform line endpoints to current joint frame
                xyzl = Tn(:,:,j) * xyz_line;
                
                % and update the graphical line
                set(h.jointaxis(j), 'Xdata', xyzl(1,:), ...
                    'Ydata', xyzl(2,:), ...
                    'Zdata', xyzl(3,:));
                set(h.jointlabel(j), 'Position', xyzl(1:3,1));
            end
        end
    end

    %
    % display the wrist axes and labels
    %
    if isfield(h, 'x')
        %
        % compute the wrist axes, based on final link transformation
        % plus the tool transformation.
        %
        xv = t*[mag;0;0;1];
        yv = t*[0;mag;0;1];
        zv = t*[0;0;mag;1];

        %
        % update the line segments, wrist axis and links
        %
        set(h.x,'xdata',[t(1,4) xv(1)], 'ydata', [t(2,4) xv(2)], ...
            'zdata', [t(3,4) xv(3)]);
        set(h.y,'xdata',[t(1,4) yv(1)], 'ydata', [t(2,4) yv(2)], ...
             'zdata', [t(3,4) yv(3)]);
        set(h.z,'xdata',[t(1,4) zv(1)], 'ydata', [t(2,4) zv(2)], ...
             'zdata', [t(3,4) zv(3)]);
        set(h.xt, 'Position', xv(1:3));
        set(h.yt, 'Position', yv(1:3));
        set(h.zt, 'Position', zv(1:3));
    end
