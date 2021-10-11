%IHIST Image histogram
%
% IHIST(IM, OPTIONS) displays the image histogram.  For an image with  multiple
% planes the histogram of each plane is given in a separate subplot.
%
% H = IHIST(IM, OPTIONS) is the image histogram as a column vector.  For
% an image with multiple planes H is a matrix with one column per image plane.
%
% [H,X] = IHIST(IM, OPTIONS) as above but also returns the bin coordinates as
% a column vector X.
%
% Options::
% 'nbins'     number of histogram bins (default 256)
% 'cdf'       compute a cumulative histogram
% 'normcdf'   compute a normalized cumulative histogram, whose maximum value
%             is one
% 'sorted'    histogram but with occurrence sorted in descending magnitude
%             order.  Bin coordinates X reflect this sorting.
%
% Example::
%
%    [h,x] = ihist(im);
%    bar(x,h);
%
%    [h,x] = ihist(im, 'normcdf');
%    plot(x,h);
%
% Notes::
% - For a uint8 image the MEX function FHIST is used (if available)
%   - The histogram always contains 256 bins
%   - The bins spans the greylevel range 0-255.
% - For a floating point image the histogram spans the greylevel range 0-1.
% - For floating point images all NaN and Inf values are first removed.
%
% See also HIST.



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

function [h,xbin] = ihist(im, varargin)

    if size(im, 3) > 1
        % color or multiband image

        np = size(im,3);
        if nargout == 0
            for k=1:np
                subplot(np, 1, k);
                ihist(im(:,:,k), varargin{:});
                xlabel(sprintf('Image plane %d', k))
                ylabel('N');
            end
        else
            for k=1:np
                [H(:,k),xx] = ihist(im(:,:,k), varargin{:});
            end
            if nargout == 1
                h = H;
            elseif nargout == 2
                h = H;
                x = xx;
            end
        end
        return
    end

    opt.nbins = 256;
    opt.type = {'hist', 'cdf', 'normcdf', 'sorted'};

    [opt,args] = tb_optparse(opt, varargin);

    if isa(im, 'uint8') && exist('fhist', 'file')
        % use quick mex function if data is integer
        [n,x] = fhist(im);
    elseif isinteger(im)
        [n,x] = hist(idouble(im(:)), opt.nbins);
        n = n'; x = x';

    else
        % remove NaN and Infs from floating point data
        z = im(:);
        k = find(isnan(z));
        z(k) = [];
        if length(k) > 0
            warning('%d NaNs removed', length(k));
        end
        k = find(isinf(z));
        z(k) = [];
        if length(k) > 0
            warning('%d Infs removed', length(k));
        end
        [n,x] = hist(z, opt.nbins);
        n = n'; x = x';
    end

    % handle options
    switch opt.type
    case 'cdf'
        n = cumsum(n);
    case 'normcdf'
        n = cumsum(n);
        n = n ./ n(end);
    case 'sorted'
        [n,i] = sort(n, 'descend');
        x = x(i);
    end

	if nargout == 0
        switch opt.type
        case {'cdf','normcdf'}
            % CDF is plotted as line graph
            plot(x, n, args{:});
            if min(size(im)) > 1
                xlabel('Greylevel')
            end
            ylabel('CDF');
        otherwise
            % histogram is plotted as bar graph
            bar(x, n, args{:});
            xaxis(min(x), max(x));
            xlabel('Greylevel')
            ylabel('Number of pixels');
        end
	elseif nargout == 1
		h = n;
	elseif nargout == 2
		h = n;
		xbin = x;
	end
