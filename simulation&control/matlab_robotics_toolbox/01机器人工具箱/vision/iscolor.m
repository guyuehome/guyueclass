%ISCOLOR Test for color image
%
% ISCOLOR(IM) is true (1) if IM is a color image, that is, it its third
% dimension is equal to three.


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

function s = iscolor(im)
    % WxH is mono
    % WxHx3 is color
    % WxHxN is mono sequence
    % WxHx3xN is color sequence
    s = isnumeric(im) && size(im,1) > 1 && size(im,2) > 1 && size(im,3) == 3;
