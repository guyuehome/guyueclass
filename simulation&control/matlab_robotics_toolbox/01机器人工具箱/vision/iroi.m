%IROI Extract region of interest
%
% OUT = IROI(IM,RECT) is a subimage of the image IM described by the 
% rectangle RECT=[umin,umax; vmin,vmax].
%
% OUT = IROI(IM,C,S) as above but the region is centered at C=(U,V) and
% has a size S.  If S is scalar then W=H=S otherwise S=(W,H).
%
% OUT = IROI(IM) as above but the image is displayed and the user is 
% prompted to adjust a rubber band box to select the region of interest.
%
% [OUT,RECT] = IROI(IM) as above but returns the coordinates of the
% selected region of interest RECT=[umin umax;vmin vmax].
%
% [OUT,U,V] = IROI(IM) as above but returns the range of U and V coordinates
% in the selected region of interest, as vectors.
%
% Notes::
% - If no output argument is specified then the result is displayed in
%   a new window.
%
% See also IDISP.

% TODO
%   IROI(image, centre, width)
%   IROI(image, [], width)     prompts to pick the centre point
%


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
function [out, out2, out3] = iroi(image, reg, wh)

    if nargin == 3
        xc = reg(1); yc = reg(2);
        if length(wh) == 1
            w = round(wh/2);
            h = w;
        else
            w = round(wh(1)/2); h = round(h(2)/2);
        end
        left = xc-w; right = xc+w;
        top = yc-h; bot = yc+h;
    elseif nargin == 2
        left = reg(1,1); right = reg(1,2);
        top = reg(2,1); bot = reg(2,2);
    else
        clf
        idisp(image, 'nogui');

        % get the rubber band box
        [p1, p2] = pickregion();
        cp0 = floor( p1 );
        cp1 = floor( p2 );

        % determine the bounds of the ROI
        top = cp0(1,2);
        left = cp0(1,1);
        bot = cp1(1,2);
        right = cp1(1,1);
        if bot<top,
            t = top;
            top = bot;
            bot = t;
        end
        if right<left,
            t = left;
            left = right;
            right = t;
        end
    end
    
    % extract the ROI
    roi = image(top:bot,left:right,:);
    if nargout == 0
        figure
        idisp(roi);
        title(sprintf('ROI (%d,%d) %dx%d', left, top, right-left, bot-top));
    else
        out = roi;
    end
    
    if nargout == 2
        out2 = [left right; top bot];
    elseif nargout == 3
        out2 = [left:right];
        out3 = [top:bot];
    end

