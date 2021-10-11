%SYM2FUNC Convert a sym object to an anonymous functions
%
% While the function matlabFunction can be used for the same purpose,
% AND and OR operators when using this function cannot be made bitwise.
% This function converts AND and OR operators to be bitwise.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) func = sym2func(sym)
%
% Outputs:
%  func : Anonymous function with bitwise AND and OR operators
%
% Inputs:
%  sym : A sym object
%
% See also matlabFunction sym

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
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

function func = sym2func(sym)

func = matlabFunction(sym);
str = func2str(func);

amps = find(str == '&');
str(amps(1:2:end)) = [];

bar = find(str == '|');
str(bar(1:2:end)) = [];

func = str2func(str);

end

