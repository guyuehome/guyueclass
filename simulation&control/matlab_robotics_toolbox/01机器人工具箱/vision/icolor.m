%ICOLOR Colorize a greyscale image
%
% C = ICOLOR(IM) is a color image C (HxWx3)where each color plane is equal 
% to IM (HxW).
%
% C = ICOLOR(IM, COLOR) as above but each output pixel is COLOR (3x1) times
% the corresponding element of IM.
%
% Examples::
%
% Create a color image that looks the same as the greyscale image
%         c = icolor(im);
% each set pixel in im is set to [1 1 1] in the output.
%
% Create an rose tinted version of the greyscale image
%         c = icolor(im, colorname('pink'));
% each set pixel in im is set to [0 1 1] in the output.
%
% Notes::
% - Can convert a monochrome sequence (HxWxN) to a color image 
%   sequence (HxWx3xN).
%
% See also IMONO, COLORIZE, IPIXSWITCH.


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

function out = icolor(im, color)

    if nargin < 2
        color = [1 1 1];
    end
    
    for i=1:size(im,3)
        c = [];
        
        for k=1:numel(color)
            c = cat(3, c, im(:,:,i)*color(k));
        end
        out(:,:,:,i) = c;      
    end
