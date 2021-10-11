%KSOBEL Sobel edge detector
%
% K = KSOBEL() is the Sobel x-derivative kernel:
%           |-1  0  1|
%           |-2  0  2|
%           |-1  0  1|
%
% Notes::
% - This kernel is an effective horizontal edge detector
% - The Sobel vertical derivative is K'
%
% See also ISOBEL.



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

function k = ksobel

    k = [   -1 0 1
            -2 0 2
            -1 0 1];
