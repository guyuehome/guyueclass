%ICONCAT Concatenate images
%
% C = ICONCAT(IM,OPTIONS) concatenates images from the cell array IM.
%
% ICONCAT(IM,OPTIONS) as above but displays the concatenated images 
% using IDISP.
%
% [C,U] = ICONCAT(IM,OPTIONS) as above but also returns the vector U whose
% elements are the coordinates of the left (or top in vertical mode) edge of
% the corresponding image within the concatenated image.
%
% Options::
% 'dir',D     direction of concatenation: 'horizontal' (default) or 'vertical'.
% 'bgval',B   value of padding pixels (default NaN)
%
% Examples::
%
% Horizontally concatenate three images
%         c = iconcat({im1, im2, im3}, 'h'); 
%
% Find the first column of each of the three images
%         [c,u] = iconcat({im1, im2, im3}, 'h'); 
% where u is a 3-vector such that im3 starts in the u(3)'rd column of c.
%
% Notes::
% - The images do not have to be of the same size, and smaller images are 
%   surrounded by background pixels which can be specified.
% - Works for color or greyscale images.
% - Direction can be abbreviated to first character, 'h' or 'v'.
% - In vertical mode all images are right justified.
% - In horizontal mode all images are top justified.
%
% See also IDISP.


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

function [out,u0] = iconcat(images, dir, bgval)
% TODO: add a gap option

    opt.dir = 'h';
    if isinteger(images{1})
        opt.bg = 0;
    else
        opt.bg = NaN;
    end

   % image is a cell array
    width = 0;
    height = 0;
    
    if nargin < 2
        dir = 'h';
    end
    if nargin < 3
        bgval = NaN;
    end
    
    np_prev = NaN;
    u0 = 1;
    for i=1:length(images)
        if dir(1) == 'v'
            images{i} = images{i}';
        end
        
        image = images{i};

        [nr,nc,np] = size(image);
        if ~isnan(np_prev) && np ~= np_prev
            error('all images must have same number of planes');
        end
        width = width + nc;
        height = max(height, nr);
        if i ~= length(images)
            u0(i+1) = u0(i) + nc;
        end
    end
    composite = bgval*ones(height, width, np, class(images{1}));

    u = 1;
    for i=1:length(images)
        composite = ipaste(composite, images{i}, [u 1]);
        u = u + size(images{i}, 2);
    end
    
    if dir == 'v'
        composite = permute(composite, [2 1 3]);
    end
    
    
    
    if nargout == 0
        idisp(composite)
    else
        out = composite;
    end
