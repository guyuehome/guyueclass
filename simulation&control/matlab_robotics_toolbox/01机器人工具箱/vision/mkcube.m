%MKCUBE Create cube
%
% P = MKCUBE(S, OPTIONS) is a set of points (3x8) that define the 
% vertices of a cube of side length S and centred at the origin.
%
% [X,Y,Z] = MKCUBE(S, OPTIONS) as above but return the rows of P as three 
% vectors.
%
% [X,Y,Z] = MKCUBE(S, 'edge', OPTIONS) is a mesh that defines the edges of
% a cube.
%
% Options::
% 'facepoint'    Add an extra point in the middle of each face, in this case
%                the returned value is 3x14 (8 vertices + 6 face centres).
% 'centre',C     The cube is centred at C (3x1) not the origin
% 'T',T          The cube is arbitrarily transformed by the homogeneous 
%                transform T
% 'edge'         Return a set of cube edges in MATLAB mesh format rather
%                than points.
%
% See also CYLINDER, SPHERE.



% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

function [o1,o2,o3] = mkcube(s, varargin)
    
    opt.centre = [];
    opt.T = [];
    opt.edge = false;
    opt.facepoint = false;

    opt = tb_optparse(opt, varargin);

    % vertices of a unit cube with one corner at origin
    cube = [
       -1    -1     1     1    -1    -1     1     1
       -1     1     1    -1    -1     1     1    -1
       -1    -1    -1    -1     1     1     1     1 ];

    if opt.facepoint
        faces = [
          1    -1     0     0     0     0
          0     0     1    -1     0     0
          0     0     0     0     1    -1 ];
        cube = [cube faces];
    end

    % vertices of cube about the origin
    cube = cube / 2 * s;

    % offset it
    if ~isempty(opt.centre)
        cube = bsxfun(@plus, cube, opt.centre(:));
    end
    % optionally transform the vertices
    if ~isempty(opt.T)
        if isvec(opt.T)
            opt.T = transl(opt.T);
        end
        cube = homtrans(opt.T, cube);
    end

    if opt.edge == false
        if nargout <= 1,
            o1 = cube;
        elseif nargout == 3,
            o1 = cube(1,:);
            o2 = cube(2,:);
            o3 = cube(3,:);
        end
    else
        cube = cube(:,[1:4 1 5:8 5]);
        o1 = reshape(cube(1,:), 5, 2)';
        o2 = reshape(cube(2,:), 5, 2)';
        o3 = reshape(cube(3,:), 5, 2)';
    end
