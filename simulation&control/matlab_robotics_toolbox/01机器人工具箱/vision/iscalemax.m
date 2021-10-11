%ISCALEMAX Scale space maxima
%
% F = ISCALEMAX(L, S) is a vector of ScalePointFeature objects which are
% the maxima, in space and scale, of the Laplacian of Gaussian (LoG) 
% scale-space image sequence L (HxWxN).  S (Nx1) is a vector of scale values 
% corresponding to each plane of L.
%
% If the pixels are considered as cubes in a larger volume, the maxima are
% those cubes greater than all their 26 neighbours.
%
% Notes::
% - Features are sorted into descending feature strength.
%
% See also ISCALESPACE, ScalePointFeature.


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

function features = iscalemax(L, sscale)

    allmax = zeros(size(L));
    nbmax = zeros(size(L));
    se_all = ones(3,3);
    se_nb = se_all; se_nb(2,2) = 0;

    % get maxima at each level
    for k=1:size(L,3)
        allmax(:,:,k) = imorph(abs(L(:,:,k)), se_all, 'max', 'replicate');
        nbmax(:,:,k) = imorph(abs(L(:,:,k)),  se_nb, 'max', 'replicate');
    end

    z = zeros(size(L,1), size(L,2));
    fc = 1;
    strength = zeros(size(L,1), size(L,2));
    for k=2:size(L,3)-1     % maxima cant be at either end of the scale range
        s = abs(L(:,:,k));
        corners = find( s > nbmax(:,:,k) & s > allmax(:,:,k-1) & s > allmax(:,:,k+1) );
        for corner=corners'
            [y,x] = ind2sub(size(s), corner);
            features(fc) = ScalePointFeature(x, y, s(corner));
            features(fc).scale_ = sscale(k)*sqrt(2);
            fc = fc+1;
        end
    end

    % sort into descending order of strength
    [z,k] = sort(-features.strength);
    features = features(k);
