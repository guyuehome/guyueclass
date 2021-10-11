%ISTRETCH Image normalization
%
% OUT = ISTRETCH(IM, OPTIONS) is a normalized image in which all pixel 
% values lie in the range 0 to 1.  That is, a linear mapping where the 
% minimum value of IM is mapped to 0 and the maximum value of IM is 
% mapped to 1.
%
% Options::
% 'max',M     Pixels are mapped to the range 0 to M
% 'range',R   R(1) is mapped to zero, R(2) is mapped to 1 (or max value).
%
% Notes::
% - For an integer image the result is a double image in the range 0 to max
%   value.
%
% See also INORMHIST.


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


function zs = istretch(z, varargin)

    opt.max = 1;
    opt.range = [];
    opt = tb_optparse(opt, varargin);

    vals = z(:);
    vals(isinf(vals)) = [];
    
    if isempty(opt.range)
        mn = min(vals);
        mx = max(vals);
    else
        mn = opt.range(1);
        mx = opt.range(2);
    end
    

    zs = (z-mn)/(mx-mn)*opt.max;

    if ~isempty(opt.range)
        zs = max(0, min(opt.max, zs));
    end