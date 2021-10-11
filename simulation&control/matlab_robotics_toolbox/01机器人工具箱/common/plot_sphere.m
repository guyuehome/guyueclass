%PLOT_SPHERE Draw sphere
%
% PLOT_SPHERE(C, R, LS) draws spheres in the current plot.  C is the 
% centre of the sphere (3x1), R is the radius and LS is an optional MATLAB 
% color spec, either a letter or a 3-vector.  
%
% H = PLOT_SPHERE(C, R, COLOR) as above but returns the handle(s) for the
% spheres.
%
% H = PLOT_SPHERE(C, R, COLOR, ALPHA) as above but ALPHA specifies the opacity
% of the sphere were 0 is transparant and 1 is opaque.  The default is 1.
%
% If C (3xN) then N sphhere are drawn and H is Nx1.  If R (1x1) then all
% spheres have the same radius or else R (1xN) to specify the radius of
% each sphere.
%
% Example::
% Create four spheres
%         plot_sphere( mkgrid(2, 1), .2, 'b')
% and now turn on a full lighting model
%         lighting gouraud
%         light
%
% NOTES::
% - The sphere is always added, irrespective of figure hold state.
% - The number of vertices to draw the sphere is hardwired.

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

% TODO
% inconsistant call format compared to other plot_xxx functions.

function h = plot_sphere(c, r, varargin)

    opt.color = 'b';
    opt.alpha = 1;
    opt.mesh = 'none';

    [opt,args] = tb_optparse(opt, varargin);
    
    % backward compatibility with RVC
    if length(args) > 0
        opt.color = args{1};
    end
    if length(args) > 1
        opt.alpha = args{2};
    end
    
    daspect([1 1 1])
    hold_on = ishold
    hold on
    [xs,ys,zs] = sphere(40);

    if isvec(c,3)
        c = c(:);
    end
    if size(r) == 1
        r = r * ones(numcols(c),1);
    end

    if nargin < 4
        alpha = 1
    end

    % transform the sphere
    for i=1:numcols(c)
        x = r(i)*xs + c(1,i);
        y = r(i)*ys + c(2,i);
        z = r(i)*zs + c(3,i);
                
        % the following displays a nice smooth sphere with glint!
        h = surf(x,y,z, 'FaceColor', opt.color, 'EdgeColor', opt.mesh, 'FaceAlpha', opt.alpha);
        % camera patches disappear when shading interp is on
        %h = surfl(x,y,z)
    end
    %lighting gouraud
    %light
    if ~hold_on
        hold off
    end
