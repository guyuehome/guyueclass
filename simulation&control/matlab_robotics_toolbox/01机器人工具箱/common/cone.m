
% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
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
%
% http://www.petercorke.com
function arrow(p0, p1, varargin)
    
    opt.scale = [];
    opt.length = 5;
    opt.n = 20;
    opt.thick = 2;
    opt.color = 'b';
    
    [opt,args] = tb_optparse(opt, varargin);
    
    if isempty(opt.scale)
        a = axis();
        opt.scale = 0.05 * max([a(2)-a(1) a(4)-a(3) a(6)-a(5)]);
    end
    
    
    % shape factors
    s = opt.scale;
    h = opt.length;       % aspect ratio, more is pointier

    % draw the line
        hold on

    plot2([p0; p1], 'LineWidth', opt.thick, 'Color', opt.color);
    
    
    % compute transform to orientation of cone base
    v1 = [0 0 1];       % xy plane circle normal
    v2 = unit(p1-p0);   % axis of line
    th = acos(v1*v2');  % angle between these vector
    % transform
    T = transl(p1-h*s*v2) * angvec2tr(th, cross(v1,v2))
    
    % scaled circle
    p = circle([0 0], 1, 'n', opt.n)*s;
    p = [p p(:,1)];
    z = ones(1,numcols(p));
    
    % compute 3D points of circle
    p = homtrans(T, [p; 0*z]);
    
    % build the mesh matrices
    %  row 1: circle points
    %  row 2: the vertex
    X = [p(1,:); p1(1)*z];
    Y = [p(2,:); p1(2)*z];
    Z = [p(3,:); p1(3)*z];
    
    % display it
    surf(X,Y,Z, 'FaceColor', opt.color, 'EdgeColor', opt.color)
    %axis equal
    
    hold off