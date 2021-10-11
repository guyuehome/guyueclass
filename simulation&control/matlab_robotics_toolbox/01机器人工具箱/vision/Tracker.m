%Tracker Track points in image sequence
%
% This class assigns each new feature a unique identifier and tracks it
% from frame to frame until it is lost.  A complete history of all
% tracks is maintained.
%
% Methods::
% plot           Plot all tracks
% tracklengths   Length of all tracks
%
% Properties::
% track     A vector of structures, one per active track.
% history   A vector of track history structures with elements id and uv
%           which is the path of the feature.
%
% See also PointFeature.


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

% TODO 
%   more wrapper funcs for track structs
%   fix idisp delete/recreate thing
%   add step and run methods?
%   add char/display methods
%   make track a vector of objects not structs?

classdef Tracker
    properties
        history
        track
        id      % unique id
        N
        thresh
        dims
    end

    methods
        function t = Tracker(im, c, varargin)
        %Tracker.Tracker Create new Tracker object
        %
        % T = Tracker(IM, C, OPTIONS) is a new tracker object.  IM (HxWxS) is an
        % image sequence and C (Sx1) is a cell array of vectors of PointFeature 
        % subclass objects.  The elements of the cell array are the point features
        % for the corresponding element of the image sequence.
        %
        % During operation the image sequence is animated and the point features
        % are overlaid along with annotation giving the unique identifier of the
        % track.
        %
        % Options::
        % 'radius',R    Search radius for feature in next frame (default 20)
        % 'nslots',N    Maximum number of tracks (default 800)
        % 'thresh',T    Similarity threshold (default 0.8)
        % 'movie',M     Write the frames as images into the folder M as 
        %               with sequential filenames.
        %
        % Notes::
        % - The 'movie' options saves frames as files NNNN.png.
        % - When using 'movie' option ensure that the window is fully visible.
        % - To convert frames to a movie use a command like:
        %        ffmpeg -r 10 -i %04d.png out.avi
        %
        % See also PointFeature.

            opt.radius = 20;
            opt.nslots = 800;
            opt.thresh = 0.8;
            opt.movie = [];
            framenum = 0;

            opt = tb_optparse(opt, varargin);

            t.id = 1;
            t.history = [];
            t.N = opt.nslots;
            t.thresh = opt.thresh;
            t.dims = size(im);
            
            if ~isempty(opt.movie)
                mkdir(opt.movie);
                framenum = 1;
            end
            
            for slot=1:t.N
                t.track(slot).busy = false;
                t.track(slot).uv = [];
                t.track(slot).uv_h = [];
                t.track(slot).seen = 0;
                t.track(slot).lastseen = 0;
                t.track(slot).feature = [];
                t.track(slot).id = 0;
                t.track(slot).curmatch = 0;
                t.track(slot).similarity = 0;
            end
            
            for slot=1:length(c{1})
                t.track(slot).busy = true;
                t.track(slot).uv = c{1}(slot).uv;
                t.track(slot).uv_h = [];
                t.track(slot).seen = 1;
                t.track(slot).lastseen = 1;
                t.track(slot).feature = c{1}(slot);
                t.track(slot).id = t.id;
                t.id = t.id + 1;
                t.track(slot).curmatch = slot;
                t.track(slot).similarity = 0;
            end

            for frame=2:length(c)
                corners = c{frame};
                
                retired = 0;

                for slot=1:t.N
                    % for all active elements in track, look for a match in the
                    % current image
                    if ~t.track(slot).busy
                        continue;
                    end

                    % compute distance from this feature to all new ones
                    if length(corners) == 0
                        continue;  % no features left to assign
                    end
                    d = distance(t.track(slot).uv, [corners.uv]);

                    % choose those that are close
                    near = find(d < opt.radius);
                    
                    
                    if length(near) > 0
                        % if there are some close by points, check their similarity
                        sim = t.track(slot).feature.ncc( corners(near) );
                        [z,best] = max(sim);
                    
                        if  z > t.thresh
                            % if the match is good, update the track
                            t.track(slot).curmatch = near(best);
                            t.track(slot).similarity = z;
                            t.track(slot).uv_h = [t.track(slot).uv_h t.track(slot).uv];
                            t.track(slot).uv = corners(near(best)).uv;
                            t.track(slot).seen = t.track(slot).seen + 1;
                            t.track(slot).lastseen = frame;
                            t.track(slot).feature = corners(near(best));  % update the feature
                            
                            corners(near(best)) = [];        % remove this corner from consideration
                        end

                    end
                    
                    if (frame - t.track(slot).lastseen) > 5
                        %fprintf('id %d lost at age %d\n', track(slot).id, track(slot).seen);
                        t.track(slot).busy = false;
                        retired = retired + 1;

                        if t.track(slot).seen > 5
                            % retire the track
                            h =[];
                            h.id = t.track(slot).id;
                            h.uv = [t.track(slot).uv_h t.track(slot).uv];
                            t.history = [t.history h];
                        end
                    end
                end
                
                fprintf('%d continuing tracks, %d new tracks, %d retired\n', ...
                    sum([t.track.busy]), length(corners), retired);
                
                for i=1:length(corners)
                    slot = find([t.track.busy]==false);
                    if length(slot) == 0
                        warning('no free slots');
                        continue;
                    end
                    slot = slot(1);
                    
                    t.track(slot).busy = true;
                    t.track(slot).uv = corners(i).uv;
                    t.track(slot).uv_h = [];
                    t.track(slot).seen = 1;
                    t.track(slot).feature = corners(i);
                    t.track(slot).id = t.id;
                    t.id = t.id + 1;
                    t.track(slot).curmatch = i;
                    t.track(slot).similarity = 0;
                    % for all remaining corners, start new track
                end
                
                k = [t.track.busy] == true & [t.track.seen] > 2 & (frame-[t.track.lastseen]) == 0;
                idisp(im(:,:,frame), 'nogui');
                plot_point( [t.track(k).uv], 'ws', 'printf', {'%d', [t.track(k).id]});
                title( sprintf('frame %d', frame) );
                drawnow

                if ~isempty(opt.movie)
                    f = getframe;
                    imwrite(f.cdata, sprintf('%s/%04d.png', opt.movie, framenum));
                    framenum = framenum+1;
                end
                
                %pause
            end

            % add curent tracks to the history
            for slot=1:t.N
                if t.track(slot).busy && t.track(slot).seen > 5
                    % retire the track
                    h =[];
                    h.id = t.track(slot).id;
                    h.uv = [t.track(slot).uv_h t.track(slot).uv];
                    t.history = [t.history h];
                end
            end


        end %constructor

        function l = tracklengths(t)
        %Tracker.tracklengths Length of all tracks
        %
        % T.tracklengths() is a vector containing the length of every track.
            l = [];
            for h=t.history
                l = [l numcols(h.uv)];
            end
        end

        function plot(t)
        %Tracker.plot Show feature trajectories
        %
        % T.plot() overlays the tracks of all features on the current plot.
            clf
            hold on
            for h=t.history
                % for each track
                plot(h.uv(1,:)', h.uv(2,:)');
            end
            axis([0 t.dims(2) 0 t.dims(1)]);
            set(gca, 'Ydir', 'reverse');
            hold off
        end

        function display(t)
        %Tracker.display Display value
        %
        % T.display() displays a compact human-readable string representation of the 
        % Tracker object
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a Tracker object and the command has no trailing
        %   semicolon.
        %
        % See also Tracker.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(t))
            if loose
                disp(' ');
            end
        end

        function s = char(t)
        %Tracker.char Convert to string
        %
        % S = T.char() is a compact string representation of the Tracker parameters and status.

            s = sprintf('Tracker: %d tracks, next id=%d, thresh=%g, nhistory=%d', ...
                t.N, t.id, t.thresh, numel(t.history));
        end

    end %methods
end %class
