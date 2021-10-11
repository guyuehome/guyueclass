%KTRIANGLE Triangular kernel
%
% K = KTRIANGLE(W) is a triangular kernel within a rectangular matrix K.  The
% dimensions K are WxW if W is scalar or W(1) wide and W(2) high.  The triangle
% is isocles and is full width at the bottom row of the kernel and with its 
% apex in the top row.
%
% Examples::
%        >> ktriangle(3)
%        ans =
%        |0  1  0|
%        |0  1  0|
%        |1  1  1|
%
% See also KCIRCLE.


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

function s = ktriangle(sz)

    sz
    if numel(sz) == 1
        w = bitor(sz, 1);  % make it odd
        h = w;
        s = zeros(w, w);

        w2 = ceil(w/2);
        h2 = w2;
    elseif numel(w) == 2
        w = bitor(sz(1), 1);  % make it odd
        h = bitor(sz(2), 1);  % make it odd
        s = zeros(h, w);

        w2 = ceil(w/2);
        h2 = ceil(h/2);
    end

    for i=1:w
        if i>w2
            y = round( ((h-1)*i + w - w2*h) /(w-w2) );
            s(y:h,i) = 1;
        else
            y = round( ((h-1)*i + 1 - w2*h) /(1-w2) );
            s(y:h,i) = 1;
        end
    end
