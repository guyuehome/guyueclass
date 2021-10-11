%IMONO Convert color image to monochrome
%
% OUT = IMONO(IM, OPTIONS) is a greyscale equivalent to the color image IM.
%
% Options::
% 'r601'       ITU recommendation 601 (default)
% 'r709'       ITU recommendation 709
% 'value'      HSV value component
%
% Notes::
% - This function returns a greyscale image whether passed a color or a
%   greyscale image.  If a greyscale image is passed it is simply returned.
% - Can convert a color image sequence (HxWx3xN) to a monochrome
%   sequence (HxWxN).
%
% See also COLORIZE, ICOLOR, COLORSPACE.



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
function out = imono(im, opt)

    % WxH is mono
    % WxHx3 is color
    % WxHxN is mono sequence
    % WxHx3xN is color sequence
    if ~iscolor(im)
        % image is already monochrome
        out = im;
        return;
    end

    if nargin < 2
        opt = 'r601';
    end

    for i=1:size(im,4)
        rgb = im(:,:,:,i);

        switch (lower(opt))
            case {'r601', 'grey', 'gray', 'mono', 'grey_601','gray_601'}
                % rec 601 luma
                out(:,:,i) = 0.299*rgb(:,:,1) + 0.587*rgb(:,:,2) + 0.114*rgb(:,:,3);
                
            case {'r709', 'grey_709','gray_709'}
                % rec 709 luma
                out(:,:,i) = 0.2126*rgb(:,:,1) + 0.7152*rgb(:,:,2) + 0.0722*rgb(:,:,3);
            case 'value'
                % 'value', the V in HSV, not CIE L*
                % the mean of the max and min of RGB values at each pixel
                mx = max(rgb, [], 3);
                mn = min(rgb, [], 3);
                if isfloat(im)
                    out = 0.5*(mx+mn);
                else
                    z = (int32(mx) + int32(mn))/2;
                    out(:,:,i) = cast(z, class(out));
                end
            otherwise
                error('unknown option');
        end
    end
