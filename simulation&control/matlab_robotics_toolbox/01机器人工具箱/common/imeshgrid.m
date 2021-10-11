%IMESHGRID Domain matrices for image
%
% [U,V] = IMESHGRID(IM) are matrices that describe the domain of image IM
% and can be used for the evaluation of functions over the image. U and V are
% the same szie as IM.  The element U(v,u) = u and V(v,u) = v.
%
% [U,V] = IMESHGRID(IM, N) as above but...
%
% [U,V] = IMESHGRID(W, H) as above but the domain is WxH.
%
% [U,V] = IMESHGRID(SIZE) as above but the domain is described size which is
% scalar SIZExSIZE or a 2-vector [W H].
%
% See also MESHGRID.


% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [U,V] = imeshgrid(a1, a2)

    if nargin == 1
        if length(a1) == 1
            % imeshgrid(S)
            % we specified a size for a square output image
            [U,V] = meshgrid(1:a1, 1:a1);
        elseif length(a1) == 2
            % imeshgrid([W H])
            % we specified a size for a rectangular output image (w,h)
            [U,V] = meshgrid(1:a1(1), 1:a1(2));
        elseif ndims(a1) >= 2
            % imeshgrid(IM)
            [U,V] = meshgrid(1:numcols(a1), 1:numrows(a1));
        else
            error('incorrect argument');
        end
    elseif nargin == 2
        if ~isscalar(a1)
            nu = 1; nv = 1;
            % imeshgrid(IM, ...)
            if length(a2) == 1
                % imeshgrid(IM, N)
                nu = a2; nv = a2;
            elseif length(a2) == 2
                % imeshgrid(IM, [NX NY])
                nu = a2(1); nv = a2(2);
            else
                error('MVTB:imeshgrid:badarg', 'bad step size');
            end
            [U,V] = meshgrid(1:nu:numcols(a1), 1:nv:numrows(a1));
            
        else
            % imeshgrid(X, Y)
            if ~isvec(a1)
                a1 = 1:a1;
            end
            if ~isvec(a2)
                a2 = 1:a2;
            end
            [U,V] = meshgrid(a1, a2);
        end
    end
    
    
