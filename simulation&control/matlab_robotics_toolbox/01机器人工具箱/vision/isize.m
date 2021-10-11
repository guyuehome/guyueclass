%ISIZE Size of image
%
% N = ISIZE(IM,D) is the size of the D'th dimension of IM.
%
% [W,H] = ISIZE(IM) is the image width W and height H.
%
% WH = ISIZE(IM) is the image size WH = [W H].
%
% [W,H,P] = ISIZE(IM) is the image width W,  height H and and number of
% planes P.  Even if the image has only two dimensions P will be one.
%
% Notes::
% - A simple convenience wrapper on the MATLAB function SIZE.
%
% See also SIZE.


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

function [o1,o2,o3] = isize(im, idx)

    if nargin == 2
        % isize(im, dim)
        o1 = size(im, idx);
    else
        % isize(im)
        s = size(im);
        if nargout == 1
            o1 = [s(2) s(1)];  % width height
        elseif nargout == 2
            o1 = s(2);  % width
            o2 = s(1);  % height, number of rows
        elseif nargout == 3
            o1 = s(2);  % width
            o2 = s(1);  % height, number of rows
            if ndims(im) == 2
                o3 = 1;
            else
                o3 = s(3);
            end
        end
    end
