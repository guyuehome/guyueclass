%D2R Convert degrees into radians
%
% Convert an angle which is expressed in degrees into radians
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) rad = d2r(deg)
%  (2) rad = d2r()
%
%  (2) is as per (1) with deg = 1
%
% Outputs:
%  rad : Angle in radians
%
% Inputs:
%  deg : Angle in degrees
%
% See also r2d

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

function rad = d2r(deg)

if ~nargin, deg = 1; end
rad = pi*deg/180;

end
