%FIREWIRE	Read from firewire camera
%
%	h = firewire(port, color, rate)
%	im = firewire(h)
%
% First form opens the interface and returns a handle or [] on error.
% Color is one of 'mono', 'rgb' or 'yuv'.  Rate is one of the
% standard DC1394 rates: 1.875, 3.75, 7.5, 15, 30 or 60 fps.  The
% highest rate less than or equal to rate is chosen.
%
% Second form reads an image.  For mono a 2-d matrix is returned,
% for rgb a 3-d matrix is returned.  For yuv a structure is
% returned with elements .y, .u and .v.
%
%
% SEE ALSO:	webcam, idisp



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
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.
