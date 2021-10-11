%IPASTE Paste an image into an image
%
% OUT = IPASTE(IM, IM2, P, OPTIONS) is the image IM with the subimage IM2 
% pasted in at the position P=[U,V].
%
% Options::
% 'centre'   The pasted image is centred at P, otherwise P is the top-left 
%            corner of the subimage in IM (default)
% 'zero'     the coordinates of P start at zero, by default 1 is assumed
% 'set'      IM2 overwrites the pixels in IM (default)
% 'add'      IM2 is added to the pixels in IM
% 'mean'     IM2 is set to the mean of pixel values in IM2 and IM
%
% Notes::
% - Pixels outside the pasted in region are unaffected.
%
% See also ILINE.


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

function out = ipaste(canvas, pattern, topleft, varargin)

    [h,w,nc] = size(canvas);
    [ph,pw,np] = size(pattern);

    opt.centre = false;
    opt.zero = false;
    opt.mode = {'set', 'add', 'mean'};

    opt = tb_optparse(opt, varargin);

    if opt.centre
        % specify centre of pattern not topleft
        left = topleft(1) - floor(pw/2);
        top = topleft(2) - floor(ph/2);
    else
        left = topleft(1);      % x
        top = topleft(2);       % y
    end

    if opt.zero
        left = left+1;
        top = top+1;
    end

    if (top+ph-1) > h
        error('pattern falls off bottom edge');
    end
    if (left+pw-1) > w
        error('pattern falls off right edge');
    end

    if np > nc
        % pattern has multiple planes, replicate the canvas
        out = repmat(canvas, [1 1 np]);
    else
        out = canvas;
    end
    if np < nc
        pattern = repmat(pattern, [1 1 nc]);
    end
    switch opt.mode
    case 'set'
        out(top:top+ph-1,left:left+pw-1,:) = pattern;
    case 'add'
        out(top:top+ph-1,left:left+pw-1,:) = out(top:top+ph-1,left:left+pw-1,:) +pattern;
    case 'mean'
        old = out(top:top+ph-1,left:left+pw-1,:);
        k = ~isnan(pattern);
        old(k) = 0.5 * (old + pattern);
        out(top:top+ph-1,left:left+pw-1,:) = old;
    end
