%Camera.plot_camera Display camera icon in world view
%
% C.plot_camera(OPTIONS) draw a camera as a simple 3D model in the current
% figure.
%
% Options::
% 'Tcam',T     Camera displayed in pose T (homogeneous transformation 4x4)
% 'scale',S    Overall scale factor (default 0.2 x maximum axis dimension)
% 'color',C    Camera body color (default blue)
% 'frustrum'   Draw the camera as a frustrum (pyramid mesh)
% 'solid'      Draw a non-frustrum camera as a solid (default)
% 'mesh'       Draw a non-frustrum camera as a mesh
% 'label'      Show the camera's name next to the camera
%
% Notes::
% - The graphic handles are stored within the Camera object.
% - A line between the red faces is parallel to the x-axis, between the
%   green faces is parallel to the y-axis.

function h = plot_camera(c, varargin)
    
    opt.Tcam = c.T;
    opt.scale = [];
    opt.frustrum = false;
    opt.label = false;
    
    [opt,arglist] = tb_optparse(opt, varargin);
    
    if isempty(opt.scale)
        % get the overall scale factor from the existing graph
        sz = [get(gca, 'Xlim'); get(gca, 'Ylim'); get(gca, 'Zlim')];
        sz = max(sz(:,2)-sz(:,1));
        opt.scale = sz / 15;
    end
    
    if ishandle(c.h_visualize)
        % if a handle already exists just update the transform
        set(c.h_visualize, 'Matrix', opt.Tcam);

    else
        % otherwise draw the graphical object from scratch in this figure
        
        if opt.frustrum
            
            % old representation as a colored pyramid
            % define pyramid dimensions from the size parameter
            w = opt.scale;
            l = w*2;
            
            hg = hgtransform;
            
            % define the vertices of the camera
            vertices = [
                0 w/2 -w/2  -w/2  w/2
                0 w/2  w/2  -w/2 -w/2
                0 l    l     l    l ];
            
            ud.vertices = vertices;
            
            % create the camera
            %vertices = homtrans(opt.Tcam, vertices);
            
            % the first index for each face controls the face color
            faces = [
                1 2 5 NaN
                2 1 3 NaN
                3 4 1 NaN
                4 1 5 NaN
                %5 2 3 4
                %2 3 4 5
                ];
            
            colors = [
                1 0 0       % R
                0 1 0       % G
                1 0 0       % R
                0 1 0       % G
                %0 0 1       % B
                ];
            
            patch('Vertices', vertices', ...
                'Faces', faces, ...
                'FaceVertexCData', colors, ...
                'FaceColor','flat', ...
                'UserData', ud, ...
                'Parent', hg);
            %set(h, 'FaceAlpha', 0.5);
            if opt.label
                text(0, 0, 0, c.name, 'Parent', hg);
            end
            c.h_visualize = hg;  % save handle for later
            
        else
            % draw a somewhat detail camera-looking object specific to the
            % camera subclass
            args = {'scale', opt.scale};
            if opt.label
                args = [args 'label'];
            end
            args = [args arglist{:}];
            c.h_visualize = c.drawCamera(args{:});
        end
        set(c.h_visualize, 'Matrix', opt.Tcam);
        
        xlabel('X'); ylabel('Y'); zlabel('Z');
    end
end
