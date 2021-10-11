%DTRANSFORM Distance transform
%
% DT = DTRANSFORM(IM, OPTIONS) is the distance transform of the 
% binary image IM. The value of each output pixel is the distance (pixels)
% to the closest set pixel.
%
% Options::
% 'Euclidean'   use Euclidean distance (default)
% 'cityblock'   use cityblock (Manhattan) distance
% 'show',T      display the evolving distance transform, with a delay of T
%               seconds between frames
%
% See also IMORPH, DISTANCEXFORM, DXform.


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

function d = dtransform(world, varargin)

    opt.metric = {'Euclidean', 'cityblock'};
    opt.show = [];

    [opt,args] = tb_optparse(opt, varargin);
    if length(args) > 0 && isnumeric(args{1})
        opt.show = args{1};
    end

    if strcmpi(opt.metric, 'cityblock')
        m = ones(3,3);
        m(2,2) = 0;
    elseif strcmpi(opt.metric, 'Euclidean')
        r2 = sqrt(2);
        m = [r2 1 r2; 1 0 1; r2 1 r2];
    end

    world(world==0) = Inf;
    world(world==1) = 0;

    count = 0;
    while 1
        world = imorph(world, m, 'plusmin');
        count = count+1;
        if opt.show
            cmap = gray(256);
            cmap = [1 0 0; cmap];
            colormap(cmap)
            image(world+1, 'CDataMapping', 'direct');
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            pause(opt.show);
        end

        if length(find(world(:)==Inf)) == 0
            break;
        end
    end

    if opt.show
        fprintf('%d iterations\n', count);
    end

    d = world;
