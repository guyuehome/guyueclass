%OTSU  Threshold selection
%
% T = OTSU(IM) is an optimal threshold for binarizing an image with a bimodal
% intensity histogram.  T is a scalar threshold that maximizes the variance 
% between the classes of pixels below and above the thresold T.
%
% Example::
%     t = otsu(im);
%     idisp(im >= t);
%
% Options::
% 'levels',N    Number of grey levels to use if image is float (default 256)
% 'valley',S    Standard deviation for the Gaussian weighted valley emphasis option
%
% Notes::
% - Performance for images with non-bimodal histograms can be quite poor.
%
% Reference::
%  A Threshold Selection Method from Gray-Level Histograms,
%  N. Otsu
%  IEEE Trans. Systems, Man and Cybernetics
%  Vol SMC-9(1), Jan 1979, pp 62-66
%
%  An improved method for image thresholding on the valley-emphasis method
%  H-F Ng, D. Jargalsaikhan etal
%  Signal and Info Proc. Assocn. Annual Summit and Conf (APSIPA)
%  2013
%  pp 1-4
%  
%
% See also NIBLACK, ITHRESH.


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

function t = otsu(im, varargin)

    opt.levels = 256;
    opt.valley = 0;
    opt = tb_optparse(opt, varargin);
    
    n = prod(size(im));
    nb = 0;
    no = n;
    ub = 0;

    % convert image to discrete values [0,N-1]
    if isfloat(im)
        N = opt.levels;
        im2 = round(im*(N-1));
    else
        im2 = im;
        N = double(intmax(class(im))) + 1;
    end

    % notation as per the Otsu paper
    %
    % uo \mu_o object
    % ub \mu_b background
    % no number of pixels in object
    % nb number of pixels in background
    % s2b  \sigma^2_b
    h = histc(im2(:), 0:N);
    uo = sum(im2(:))/n;

    % the between class variance
    s2b = zeros(N,1);
    for T=1:N

        nt = h(T);
        nb_new = nb + nt;
        no_new = no - nt;

        if (nb_new == 0) || (no_new == 0)
            continue;
        end

        ub = (ub*nb + nt*(T-1)) / nb_new;
        uo = (uo*no - nt*(T-1)) / no_new;
        
        if opt.valley == 0
            W = 1;
        else
            % valley emphasis option
            W = 0;
            for i=1:N
                W =  W + h(i)/n * exp((i-T)/2/opt.valley^2);
            end
            W = 1 - W;
        end

        s2b(T) = W * nb*no*(ub - uo)^2;

        %fprintf('%d %d %f %f %f\n', nb, no, ub, uo, s2b(T));
        nb = nb_new;
        no = no_new;

    end

    [z,t] = max(s2b);

    if isfloat(im)
        t = t / N;
    end
