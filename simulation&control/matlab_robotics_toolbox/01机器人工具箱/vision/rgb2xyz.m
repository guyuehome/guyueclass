%RGB2XYZ	Convert RGB to XYZ color space
%
%	[x, y, z] = RGB2XYZ(r, g, b)
%	xyz = RGB2XYZ(rgb)
%
%	Convert (R,G,B) coordinates to (X,Y,Z) color space.
%	If RGB (or R, G, B) have more than one row, then computation is
% 	done row wise.
%
% SEE ALSO:	ccxyz cmfxyz
%

% Copyright (C) 1995-2009, by Peter I. Corke
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
function [x,y,z] = rgb2xyz(R,G,B)

	a = [
		3.40479 -1.537150 -0.498535
		-0.969256 1.875992 0.041556
		0.055648 -0.204043 1.057311];
	ai = inv(a);

	if nargin == 3
		RGB = [R G B];
	elseif nargin == 1,
		RGB = R;
	else
		error('wrong number of arguments')
	end

    if isinteger(RGB)
        RGB = double(RGB);
    end

	XYZ = [];
    XYZ = (ai*RGB')';
% 	for rgb = RGB',
% 		xyz = ai*rgb;
% 		XYZ = [XYZ xyz];
% 	end
% 	XYZ = XYZ';

	if nargout == 1
		x = XYZ;
	elseif nargout == 3,
		x = XYZ(:,1);
		y = XYZ(:,2);
		z = XYZ(:,3);
	end
