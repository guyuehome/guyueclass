%R2D Convert radians into degrees
%
% Convert an angle which is expressed in radians into degrees
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) deg = r2d(rad)
%  (2) deg = r2d()
%
%  (2) is as per (1) with rad = 1
%
% Outputs:
%  deg : Angle in degrees
%
% Inputs:
%  rad : Angle in radians
%
% See also d2r

% LICENSE STATEMENT:
%
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.

function deg = r2d(rad)

if ~nargin, rad = 1; end
deg = 180*rad/pi;

end

