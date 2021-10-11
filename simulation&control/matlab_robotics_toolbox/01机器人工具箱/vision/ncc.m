%NCC Normalized cross correlation
%
% M = NCC(I1, I2) is the normalized cross-correlation between the 
% two equally sized image patches I1 and I2.  The result M is a scalar in
% the interval -1 (non match) to 1 (perfect match) that indicates similarity.
%
% Notes::
% - A value of 1 indicates identical pixel patterns.
% - The NCC similarity measure is invariant to scale changes in image
%   intensity.
%
% See also ZNCC, SAD, SSD, ISIMILARITY.



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


function m = ncc(w1, w2)

	denom = sqrt( sum(sum(w1.^2))*sum(sum(w2.^2)) );

	if denom < 1e-10,
		m = 0;
	else
		m = sum(sum((w1.*w2))) / denom;
	end
