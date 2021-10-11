%SAD Sum of absolute differences
%
% M = SAD(I1, I2) is the sum of absolute differences between the 
% two equally sized image patches I1 and I2.  The result M is a scalar that
% indicates image similarity, a value of 0 indicates identical pixel patterns
% and is increasingly positive as image dissimilarity increases.
%
% See also ZSAD, SSD, NCC, ISIMILARITY.



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

function m = sad(w1, w2)

	m = abs(w1-w2);

    m = sum(m(:));
