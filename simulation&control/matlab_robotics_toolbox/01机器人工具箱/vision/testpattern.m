%TESTPATTERN Create test images
%
% IM = TESTPATTERN(TYPE, W, ARGS) creates a test pattern image.  If W is a
% scalar the image is WxW else W(2)xW(1).  The image is specified by the
% string TYPE and one or two (type specific) arguments:
%
% 'rampx'     intensity ramp from 0 to 1 in the x-direction. ARGS is the number
%             of cycles.
% 'rampy'     intensity ramp from 0 to 1 in the y-direction. ARGS is the number
%             of cycles.
% 'sinx'      sinusoidal intensity pattern (from -1 to 1) in the x-direction. 
%             ARGS is the number of cycles.
% 'siny'      sinusoidal intensity pattern (from -1 to 1) in the y-direction. 
%             ARGS is the number of cycles.
% 'dots'      binary dot pattern.  ARGS are dot pitch (distance between 
%             centres); dot diameter.
% 'squares'   binary square pattern.  ARGS are pitch (distance between 
%             centres); square side length.
% 'line'      a line.  ARGS are theta (rad), intercept.
%   
% Examples::
%
% A 256x256 image with 2 cycles of a horizontal sawtooth intensity ramp:
%      testpattern('rampx', 256, 2);
%
% A 256x256 image with a grid of dots on 50 pixel centres and 20 pixels in
% diameter:
%      testpattern('dots', 256, 50, 25);
%
% Notes::
% - With no output argument the testpattern in displayed using idisp.
%
% See also IDISP.



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


function Z = testpattern(type, w, varargin)

    z = zeros(w);
    switch type,
    case {'sinx'}
        if nargin > 2,
            ncycles = varargin{1};
        else
            ncycles = 1;
        end
        x = 0:(numcols(z)-1);
        clength = numcols(z)/ncycles;
        z = repmat( sin(x/(clength)*ncycles*2*pi), numrows(z), 1);

    case {'siny'}
        if nargin > 2
            ncycles = varargin{1};
        else
            ncycles = 1;
        end
        clength = numrows(z)/ncycles;
        y = [0:(numrows(z)-1)]';
        z = repmat( sin(y/(clength)*ncycles*2*pi), 1, numrows(z));
        
    case {'rampx'}
        if nargin > 2
            ncycles = varargin{1};
        else
            ncycles = 1;
        end
        clength = numcols(z)/ncycles;
        x = 0:(numcols(z)-1);
        z = repmat( mod(x, clength) / (clength-1), numrows(z), 1);
        
    case {'rampy'}
        if nargin > 2
            ncycles = varargin{1};
        else
            ncycles = 1;
        end
        clength = numrows(z)/ncycles;
        y = [0:(numrows(z)-1)]';
        z = repmat( mod(y, clength) / (clength-1), 1, numcols(z));
        
    case {'line'}
        % args:
        %   angle intercept
        nr = numrows(z);
        nc = numcols(z);
        c = varargin{2};
        theta = varargin{1};

        if abs(tan(theta)) < 1,
            x = 1:nc;
            y = round(x*tan(theta) + c);
            
            s = find((y >= 1) & (y <= nr));

        else
            y = 1:nr;
            x = round((y-c)/tan(theta));
            
            s = find((x >= 1) & (x <= nc));

        end
        for k=s,    
            z(y(k),x(k)) = 1;
        end
        
    case {'squares'}
        % args:
        %   pitch diam 
        nr = numrows(z);
        nc = numcols(z);
        d = varargin{2};
        pitch = varargin{1};
        if d > (pitch/2),
            fprintf('warning: squares will overlap\n');
        end
        rad = floor(d/2);
        d = 2*rad;
        for r=pitch/2:pitch:(nr-pitch/2)
            for c=pitch/2:pitch:(nc-pitch/2),
                z(r-rad:r+rad,c-rad:c+rad) = ones(d+1);
            end
        end
        
    case {'dots'}
        % args:
        %   pitch diam 
        nr = numrows(z);
        nc = numcols(z);
        d = varargin{2};
        pitch = varargin{1};
        if d > (pitch/2)
            fprintf('warning: dots will overlap\n');
        end
        rad = floor(d/2);
        d = 2*rad;
        s = kcircle(d/2);
        for r=pitch/2:pitch:(nr-pitch/2),
            for c=pitch/2:pitch:(nc-pitch/2),
                z(r-rad:r+rad,c-rad:c+rad) = s;
            end
        end
        
    otherwise
        disp('Unknown pattern type')
        im = [];
    end

    if nargout == 0
        idisp(z);
    else
        Z = z;
    end
