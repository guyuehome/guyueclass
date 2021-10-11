%ISIMILARITY Locate template in image
%
% S = ISIMILARITY(T, IM) is an image where each pixel is the ZNCC similarity
% of the template T (MxM) to the MxM neighbourhood surrounding the
% corresonding input pixel in IM.  S is same size as IM.
%
% S = ISIMILARITY(T, IM, METRIC) as above but the similarity metric is specified
% by the function METRIC which can be any of @sad, @ssd, @ncc, @zsad, @zssd.
%
% Example::
%  Load an image of Wally/Waldo (the template)
%         T = iread('wally.png', 'double');
%  then load an image of the crowd where he is hiding
%         crowd = iread('wheres-wally.png', 'double');
%  Now search for him using the ZNCC matching measure
%         S = isimilarity(T, crowd, @zncc);
%  and display the similarity
%         idisp(S, 'colormap', 'jet', 'bar')
%  The magnitude at each pixel indicates how well the template centred on
%  that point matches the surrounding pixels.  The locations of the maxima
%  are
%         [~,p] = peak2(S, 1, 'npeaks', 5); 
%
%  Now we can display the original scene
%         idisp(crowd)
%  and highlight the most likely places that Wally/Waldo is hiding
%
%         plot_circle(p, 30, 'fillcolor', 'b', 'alpha', 0.3, ...
%           'edgecolor', 'none')
%         plot_point(p, 'sequence', 'bold', 'textsize', 24, ...
%           'textcolor', 'k', 'Marker', 'none')
%
% References::
%  - Robotics, Vision & Control, Section 12.4,
%    P. Corke, Springer 2011.
%
% Notes::
% - For NCC and ZNCC the maximum in S corresponds to the most likely template
%   location.  For SAD, SSD, ZSAD and ZSSD the minimum value corresponds
%   to the most likely location.
% - Similarity is not computed for those pixels where the template crosses 
%   the image boundary, and these output pixels are set to NaN.
% - The ZNCC function is a MEX file and therefore the fastest
% - User provided similarity metrics can be used, the function accepts
%   two regions and returns a scalar similarity score.
%
% See also IMATCH, SAD, SSD, NCC, ZSAD, ZSSD, ZNCC.



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

function S = isimilarity(T, im, metric)

%TODO add all the other similarity metrics, including rank and census

    if nargin < 3
        metric = @zncc;
    end
    [nr,nc] = size(im);
    
    if rem(numcols(T), 2) == 0 || rem(numrows(T), 2) == 0
        error('MVTB:isimilarity:badarg', 'template must have odd dimensions');
    end
    hc = floor( (numcols(T)-1)/2 );
    hr = floor( (numrows(T)-1)/2 );
    hr1 = hr+1;
    hc1 = hc+1;

    S = NaN(size(im));
    
    for c=hc1:nc-hc1
        for r=hr1:nr-hr1
            S(r,c) = metric(T, im(r-hr:r+hr,c-hc:c+hc));
        end
    end
