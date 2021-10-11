%ANAGLYPH Convert stereo images to an anaglyph image
%
% A = ANAGLYPH(LEFT, RIGHT) is an anaglyph image where the two images of
% a stereo pair are combined into a single image by coding them in two 
% different colors.  By default the left image is red, and the right 
% image is cyan.
%
% ANAGLYPH(LEFT, RIGHT) as above but display the anaglyph.
%
% A = ANAGLYPH(LEFT, RIGHT, COLOR) as above but the string COLOR describes
% the color coding as a string with 2 letters, the first for left, the second 
% for right, and each is one of:
%
%  'r'   red
%  'g'   green
%  'b'   green
%  'c'   cyan
%  'm'   magenta
%
% A = ANAGLYPH(LEFT, RIGHT, COLOR, DISP) as above but allows for disparity 
% correction.  If DISP is positive the disparity is increased, if negative it
% is reduced.  These adjustments are achieved by trimming the images.  Use 
% this option to make the images more natural/comfortable to view, useful 
% if the images were captured with a stereo baseline significantly different
% the human eye separation (typically 65mm).
%
% Example::
% Load the left and right images
%         L = iread('rocks2-l.png', 'reduce', 2);
%         R = iread('rocks2-r.png', 'reduce', 2);
% then display the anaglyph for viewing with red-cyan glasses
%         anaglyph(L, R);
%
% References::
%  - Robotics, Vision & Control, Section 14.3,
%    P. Corke, Springer 2011.
%
% See also STDISP.



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

function anaglyph = anaglyph(left, right, colors, disp)

    if nargin < 3,
        colors = 'rc';
    end
    if nargin < 4,
        disp = 0;
    end

    % ensure the images are greyscale
    left = imono(left);
    right = imono(right);

    [height,width] = size(left);
    if disp > 0,
        left = left(:,1:width-disp);
        right = right(:,disp+1:end);
    end
    if disp < 0,
        disp = -disp;
        left = left(:,disp+1:end);
        right = right(:,1:width-disp);
    end

    ag = zeros([size(left) 3]);

    ag = ag_insert(ag, left, colors(1));
    ag = ag_insert(ag, right, colors(2));

    if nargout > 0,
        aglyph = ag;
    else
        if isa(left, 'uint8'),
            ag = ag / 255;
        end
        image(ag);
    end

function out = ag_insert(in, im, c)

    out = in;
    % map single letter color codes to image planes
    switch c
    case 'r'
        out(:,:,1) = im;        % red
    case 'g'
        out(:,:,2) = im;        % green
    case 'b'
        % blue
        out(:,:,3) = im;
    case 'c'
        out(:,:,2) = im;        % cyan
        out(:,:,3) = im;
    case 'm'
        out(:,:,1) = im;        % magenta
        out(:,:,3) = im;
    case 'o'
        out(:,:,1) = im;        % orange
        out(:,:,2) = im;
    end
