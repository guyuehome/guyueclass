%IPAD Pad an image with constants
%
% OUT = IPAD(IM, SIDES, N) is a padded version of the image IM with a block 
% of NaN values N pixels wide on the sides of IM as specified by SIDES.
%
% OUT = IPAD(IM, SIDES, N, V) as above but pads with pixels of value V.
%
% SIDES is a string containing one or more of the characters:
% 't'   top
% 'b'   bottom
% 'l'   left
% 'r'   right
%
% Examples::
%
% Add a band of zero pixels 20 pixels high across the top of the image:
%     ipad(im, 't', 20, 0)
%
% Add a band of white pixels 10 pixels wide on all sides of the image:
%     ipad(im, 'tblr', 10, 255)
%
% Notes::
% - Not a tablet computer.


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

function out = ipad(in, sides, n, val)

    if nargin < 4
        val = NaN;
    end
    
    if ndims(in) > 2
        % multiplane case
        d = size(in);
        nimages = prod(d(3:end));
        out = [];
        for i=1:nimages
            out(:,:,i) = ipad(in(:,:,i), sides, n, val);
        end

        d2 = size(out);
        out = reshape(out, [d2(1) d2(2) d(3:end)]);
        return
    end
    
    out = in;
    for side=sides
        [w,h] = isize(out);

        switch side
            case 't'
                out = [val*ones(n,w); out];
            case 'b'
                out = [out; val*ones(n,w)];
            case 'l'
                out = [val*ones(h,n) out];
            case 'r'
                out = [out val*ones(h,n)];
        end
    end

                
