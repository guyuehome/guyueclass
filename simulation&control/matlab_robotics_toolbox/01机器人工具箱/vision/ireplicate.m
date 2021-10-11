%IREPLICATE  Expand image
%
% OUT = IREPLICATE(IM, K) is an expanded version of the image (HxW) where 
% each pixel is replicated into a KxK tile.  If IM is HxW the result is (KH)x(KW).
%
% See also IDECIMATE, ISCALE.


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

function ir2 = ireplicate(im, M)

    if size(im, 3) > 1
        % deal with multi-plane image
        ir2 = [];
        for i=1:size(im, 3)
            ir2 = cat(3, ir2, ireplicate(im(:,:,i), M) );
        end
        return
    end

    if nargin < 2
        M = 1;
    end

    dims = size(im);
    nr = dims(1); nc = dims(2);

    % replicate the columns
    ir = zeros(M*nr,nc, class(im));
    for i=1:M
        ir(i:M:end,:) = im;
    end

    % replicate the rows
    ir2 = zeros(M*nr,M*nc, class(im));
    for i=1:M
        ir2(:,i:M:end) = ir;
    end

