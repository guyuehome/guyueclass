% CAMCALD Camera calibration from data points
%
% C = CAMCALD(D) is the camera matrix (3x4) determined by least squares 
% from corresponding world and image-plane points.  D is a table 
% of points with rows of the form [X Y Z U V] where (X,Y,Z) is the 
% coordinate of a world point and [U,V] is the corresponding image 
% plane coordinate. 
%
% [C,E] = CAMCALD(D) as above but E is the maximum residual error after 
% back substitution [pixels]. 
% 
% Notes:
% - This method assumes no lense distortion affecting the image plane
%   coordinates.
%
% See also CentralCamera.



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



function [C, resid] = camcald(XYZ, uv)

    if numcols(XYZ) ~= numcols(uv)
        error('must have same number of world and image-plane points');
    end

    n = numcols(XYZ);
    if n < 6,
        error('At least 6 points required for calibration');
    end
%
% build the matrix as per Ballard and Brown p.482
%
% the row pair are one row at this point
%
    A = [ XYZ' ones(n,1) zeros(n,4) -repmat(uv(1,:)', 1,3).*XYZ' ...
          zeros(n,4) XYZ' ones(n,1) -repmat(uv(2,:)', 1,3).*XYZ'  ];
%
% reshape the matrix, so that the rows interleave
%
    A = reshape(A',11, n*2)';
    if rank(A) < 11,
        error('Rank deficient,  perhaps points are coplanar or collinear');
    end

    B = reshape( uv, 1, n*2)';

    C = A\B;    % least squares solution
    resid = max(max(abs(A * C - B)));
    if resid > 1,
        warning('Residual greater than 1 pixel');
    end
    fprintf('maxm residual %f pixels.\n', resid);
    C = reshape([C;1]',4,3)';
