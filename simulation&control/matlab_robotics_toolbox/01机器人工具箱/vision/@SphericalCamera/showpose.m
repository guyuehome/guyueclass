
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
function showcam(c, T, P)

    if nargin < 2
        T = c.Tcam
    end

    daspect([1 1 1])
    hold on
    [xs,ys,zs] = sphere(20);

    % transform the sphere
    xyz = transformp(T, [xs(:)'; ys(:)'; zs(:)']);
    x = reshape(xyz(1,:), size(xs));
    y = reshape(xyz(2,:), size(xs));
    z = reshape(xyz(3,:), size(xs));
            
    %surf(x,y,z, 'FaceColor', 'w', 'EdgeColor', 0.95*[1 1 1])
    surf(x,y,z, 'FaceColor', [0.8 0.8 1], 'EdgeColor', 0.5*[0.8 0.8 1], ...
        'EdgeLighting', 'gouraud')
    light
    lighting gouraud

    A = 1.6;
    o = transformp(T, [0 0 0]');
    ax = transformp(T, [A 0 0]');
    arrow3(o', ax'); text(ax(1), ax(2), ax(3), ' X')
    ax = transformp(T, [0 A 0]');
    arrow3(o', ax'); text(ax(1), ax(2), ax(3), ' Y')
    ax = transformp(T, [0 0 A]');
    arrow3(o', ax'); text(ax(1), ax(2), ax(3), ' Z')

    %grid off
    %set(gcf, 'Color', 'w');
    %set(gca,'Xcolor', 'w')
    %set(gca,'Ycolor', 'w')
    %set(gca,'Zcolor', 'w')
    %view(120, 30);

    axis
    limits = reshape(axis, 2, []);
    maxdim = max(diff(limits));
    if nargin > 1
        for i=1:numcols(P)
            plot3([o(1) P(1,i)], [o(2) P(2,i)], [o(3) P(3,i)], 'r');
            plot_sphere(P(:,i), maxdim*0.03, 'r');
        end
    end
    hold off
    xlabel('x'); ylabel('y'); zlabel('z');
