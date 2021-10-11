%ITRIM Trim images
%
% This function has two different modes of functionality.
%
% OUT = ITRIM(IM, SIDES, N) is the image IM with N pixels removed from the
% image sides as specified by SIDES which is a string containing one or 
% more of the characters:
%   't'   top
%   'b'   bottom
%   'l'   left
%   'r'   right
%
% [OUT1,OUT2] = ITRIM(IM1,IM2) returns the central parts of images IM1 and 
% IM2 as OUT1 and OUT2 respectively.  When images are rectified or warped
% the shapes can become quite distorted and are embedded in rectangular images
% surrounded by black of NaN values.  This function crops out the
% central rectangular region of each.  It assumes that the undefined pixels
% in IM1 and IM2 have values of NaN.  The same cropping is applied to each
% input image.
%
% [OUT1,OUT2] = ITRIM(IM1,IM2,T) as above but the threshold T in the range
% 0 to 1 is used to adjust the level of cropping.  The default is 0.5, a 
% higher value will include fewer NaN value in the result (smaller region),
% a lower value will include more (larger region).  A value of 0 will ensure
% that there are no NaN values in the returned region.
%
% See also HOMWARP, IRECTIFY.


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
function [out1,out2] = itrim(in1, in2, thresh)

    if ischar(in2)
        % itrim(image, edge, numpix)

        d = thresh;
        sides = in2;

        out1 = in1;
        for side=sides
            switch side
                case 't'
                    out1 = out1(d+1:end,:,:);
                case 'b'
                    out1 = out1(1:end-d,:,:);
                case 'l'
                    out1 = out1(:,d+1:end,:);
                case 'r'
                    out1 = out1(:,1:end-d,:);
            end
        end
    else
        % itrim(im1, im2, thresh)

        if nargin < 3
            thresh = 0.75;
        end
        
        out1 = trimx(in1, thresh);
        out2 = trimx(in2, thresh);
        
        z = iconcat({out1, out2});
        
        z = trimy(z, thresh);
        
        w1 = size(out1, 2);
        
        out1 = z(:,1:w1);
        out2 = z(:,w1+1:end);
        
        [w1,h1] = isize(out1);
        [w2,h2] = isize(out2);
        
        if w1 > w2
            out1 = out1(:,1:w2);
        else
            out2 = out2(:,1:w1);   
        end
    end

end % itrim
    
    
    function out = trim(in, thresh)

        out = trimx(in, thresh);


        out = trimy(out, t);
    end

    
    function out = trimx(in, thresh)
        % trim contiguous edge columns that are mostly NaN
        t = sum(isnan(in)) > thresh*size(in,1);
        
        out = in;
        n = chunk(t);
        if n > 0
            out = out(:,n+1:end);
        end
        
        n = chunk(t(end:-1:1));
        if n > 0
            out = out(:,1:end-n);
        end
    end

    function out = trimy(in, thresh)
        out = trimx(in', thresh)';
    end

    function n = chunk(t)
        n = 0;
        for i=t(:)'
            if i == 0
                break;
            else
                n = n + 1;
            end 
        end
    end

