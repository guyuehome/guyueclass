%SHOWPOSE Display a camera icon in 3D
%
%   h = cam.showpose(T)
%
%  Create a new camera at pose T, and return the graphics handle.
%
% The camera is depicted as a pyramid with the apex as the camera
% origin and the base plane normal to the optical axis.
% The sides are colored red, green, blue corresponding to the X, Y, Z axes
% respectively.


function h = showpose(c, T, sz)

    if nargin < 2
        T = eye(4,4);
    end
    if nargin < 3
        % get the overall scale factor from the existing graph
        sz = [get(gca, 'Xlim'); get(gca, 'Ylim'); get(gca, 'Zlim')];
        sz = max(sz(:,2)-sz(:,1));
        sz = sz / 20;
    end
        
    % define pyramid dimensions from the size parameter
    w = sz;
    l = sz*2;
    
    % define the vertices of the camera
    vertices = [
         0    0    0
         w/2  w/2  l
        -w/2  w/2  l
        -w/2 -w/2  l
         w/2 -w/2  l
        ];

    ud.vertices = vertices;

    % create the camera 
    vertices = transformp(T, vertices')';

    % the first index for each face controls the face color
    faces = [
        1 2 5 NaN
        2 1 3 NaN
        3 4 1 NaN
        4 1 5 NaN
        5 2 3 4
        %2 3 4 5
        ];

    colors = [
        1 0 0       % R
        0 1 0       % G
        1 0 0       % R
        0 1 0       % G
        0 0 1       % B
        ];

    if isempty(c.camview)
        % create the camera view
        h = patch('Vertices', vertices, ...
            'Faces', faces, ...
            'FaceVertexCData', colors, ...
            'FaceColor','flat', ...
            'UserData', ud);

        set(h, 'FaceAlpha', 1.0);
        c.camview = h;
        c.camview_sz = sz;
    else
        set(c.camview, 'Vertices', vertices);
    end
end
