%IANIMATE Display an image sequence
%
% IANIMATE(IM, OPTIONS) displays a greyscale image sequence IM (HxWxN) or
% a color image sequence IM (HxWx3xN) where N is the number of frames in 
% the sequence.
%
% IANIMATE(IM, FEATURES, OPTIONS) as above but with point features overlaid.
% FEATURES (Nx1) is a cell array whose elements are vectors of feature 
% objects for the corresponding frames of IM.  The feature is plotted 
% using the feature object's plot method and additional options are passed 
% through to that method.
%
% Examples::
%
% Animate image sequence:
%     ianimate(seq);
%
% Animate image sequence with overlaid corner features:
%     c = icorner(im, 'nfeat', 200);  % computer corners
%     ianimate(seq, c, 'gs');  % features shown as green squares
%
% Options::
%  'fps',F       set the frame rate (default 5 frames/sec)
%  'loop'        endlessly loop over the sequence
%  'movie',M     save the animation as a series of PNG frames in the folder M
%  'npoints',N   plot no more than N features per frame (default 100)
%  'only',I      display only the I'th frame from the sequence
%  'title',T     displays the specified title on each frame, T is a cell
%                array (1xN) of strings.
%
% Notes::
% - If titles are not specified the title is "frame N"
% - If the 'movie' is used the frames can be converted to a movie
%   using a utility like ffmpeg, for instance:
%
%          ffmpeg -i *.png -r 5 movie.mp4
%
%   or to set the bit rate explicitly
%          ffmpeg -i *.png -b:v 64k movie.mp4
%
% See also PointFeature, IHARRIS, ISURF, IDISP.


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

% TODO should work with color image sequence, dims are: row col plane seq

function ianimate(im, varargin)

    points = [];
    opt.fps = 5;
    opt.loop = false;
    opt.npoints = 100;
    opt.only = [];
    opt.movie = [];
    opt.title = [];

    [opt, arglist]  = tb_optparse(opt, varargin);

    if length(arglist) >= 1 && iscell(arglist(1))
        points = arglist{1};
        arglist = arglist(2:end);
    end
    
    clf
    pause on
    
    if ~isempty(opt.movie)
        mkdir(opt.movie);
        framenum = 1;
    end

    if ndims(im) == 3
        nframes = size(im, 3);
        color = false;
    else
        nframes = size(im, 4);
        color = true;
    end
    
    while true
        for i=1:nframes
            if opt.only ~= i
                continue;
            end
            
            if color
                image(im(:,:,:,i)); 
            else
                colormap(gray(256));
                image(im(:,:,i), 'CDataMapping', 'Scaled');
            end
            if ~isempty(points)
                f = points{i};
                n = min(opt.npoints, length(f));
                f(1:n).plot(arglist{:});
            end
            if isempty(opt.title)
                title( sprintf('frame %d', i) );
            else
                title( opt.title{i} );
            end

            if opt.only == i
                return;
            end
            if isempty(opt.movie)
                pause(1/opt.fps);
            else
                f = getframe;
                imwrite(f.cdata, sprintf('%s/%04d.png', opt.movie, framenum));
                framenum = framenum+1;    
            end
        end

        if ~opt.loop
            break;
        end
   end
