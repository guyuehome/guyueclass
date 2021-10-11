%HITORMISS Hit or miss transform
%
% H = HITORMISS(IM, SE) is the hit-or-miss transform of the binary image IM with
% the structuring element SE.  Unlike standard morphological operations S has
% three possible values: 0, 1 and don't care (represented by NaN).
%
% References::
%  - Robotics, Vision & Control, Section 12.5.3,
%    P. Corke, Springer 2011.
%
% See also IMORPH, ITHIN, ITRIPLEPOINT, IENDPOINT.


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

function hm = hitormiss(A, S1, S2)
    
    if nargin == 2
        S2 = double(S1 == 0);
        S1 = double(S1 == 1);
    end
    hm = imorph(A, S1, 'min') & imorph((1-A), S2, 'min');
