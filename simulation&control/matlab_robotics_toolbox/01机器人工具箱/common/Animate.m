%ANIMATE Create an animation
%
% Helper class for creating animations.  Saves snapshots of a figture as a
% folder of individual PNG format frames numbered 0000.png, 0001.png and so
% on.
%
% Example::
%
%          anim = Animate('movie');
%
%          for i=1:100
%              plot(...);
%              anim.add();
%          end
%
% To convert the image files to a movie you could use a tool like ffmpeg
%           % ffmpeg -r 10 -i movie/*.png out.mp4

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com


classdef Animate < handle
    properties
        frame
        dir
        resolution
    end
    
    methods
        function a = Animate(name, res)
            %ANIMATE.ANIMATE Create an animation class
            %
            % A = ANIMATE(NAME, OPTIONS) initializes an animation, and creates a folder
            % called NAME to hold the individual frames.
            %
            % Options::
            % 'resolution',R    Set the resolution of the saved image to R pixels per
            % inch.
        
            a.frame = 0;
            a.dir = name;
            mkdir(name);
            if nargin > 1
                a.resolution = res;
            else
                a.resolution = [];
            end
            delete( fullfile(name, '*.png') );
            
        end
        
        function add(a, fh)
            %ANIMATE.ADD Adds current plot to the animation
            %
            % A.ADD() adds the current figure in PNG format to the animation
            % folder with a unique sequential filename.
            %
            % A.ADD(FIG) as above but captures the figure FIG.
            %
            % See also print.

            if nargin < 2
                fh = gcf;
            end
            
            if isempty(a.resolution)
                print(fh, '-dpng', fullfile(a.dir, sprintf('%04d.png', a.frame)));
            else
                print(fh, '-dpng', sprintf('-r%d', a.resolution), fullfile(a.dir, sprintf('%04d.png', a.frame)));
            end
            a.frame = a.frame + 1;
        end
        
        function close(a)
        end
    end
end
