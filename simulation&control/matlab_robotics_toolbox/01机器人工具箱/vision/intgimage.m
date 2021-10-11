%INTIMAGE Compute integral image
%
% OUT = INTIMAGE(IM) is an integral image corresponding to IM.
%
% Integral images can be used for rapid computation of summations over 
% rectangular regions.
%
% Examples::
% Create integral images for sum of pixels over rectangular regions
%        i = intimage(im);
%
% Create integral images for sum of pixel squared values over rectangular 
% regions
%        i = intimage(im.^2);
%
% See also IISUM.


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

function ii = intgimage(I)

    ii = cumsum( cumsum(I)' )';
