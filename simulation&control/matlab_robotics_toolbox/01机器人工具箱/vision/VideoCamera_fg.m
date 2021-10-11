%VideoCamera_fg Class to read from local video camera
%
% A concrete subclass of ImageSource that acquires images from a local
% camera using a simple open-source frame grabber interface.  
%
% This class is not intended to be used directly, instead use the factory
% method VideoCamera.which will return an instance of this class if the interface
% is supported on your platform (Mac or Linux), for example
%
%         vid = VideoCamera.amera();
%
% Methods::
% grab    Aquire and return the next image
% size    Size of image
% close   Close the image source
% char    Convert the object parameters to human readable string
%
% See also ImageSource, AxisWebCamera, Movie.


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

% mmread brings the whole movie into memory.  Not entirely sure what
% libavbin uses memory-wise, it takes a long time to "open" the file.

classdef VideoCamera_fg < ImageSource

    properties

        name;
    end

    properties(Constant=true)
        % set the operation codes for the camera driver
        CAMERA_OP_LIST = 1;
        CAMERA_OP_OPEN = 2;
        CAMERA_OP_CLOSE = 3;
        CAMERA_OP_START = 4;
        CAMERA_OP_STOP = 5;
        CAMERA_OP_ISRUNNING = 6;
        CAMERA_OP_GRAB = 7;
        CAMERA_OP_GET_PARAMS = 8;
        CAMERA_OP_SET_WIDTH_HEIGHT = 9;
        CAMERA_OP_GET_NAME = 10;
    end

    methods(Static)
        % class method to list the available video sources
        function list()
            framegrabber(VideoCamera_fg.CAMERA_OP_LIST);
        end
    end

    methods

        function m = VideoCamera_fg(varargin)
        %VideoCamera_fg.VideoCamera_fg Video camera constructor
        %   
        % V = VideoCamera_fg.CAMERA, OPTIONS) is a VideoCamera_fg.object that acquires
        % images from the local video camera specified by the string CAMERA.
        %
        % If CAMERA is '?' a list of available cameras, and their
        % characteristics is displayed.
        %   
        % Options::
        % 'uint8'          Return image with uint8 pixels (default)
        % 'float'          Return image with float pixels
        % 'double'         Return image with double precision pixels
        % 'grey'           Return greyscale image
        % 'gamma',G        Apply gamma correction with gamma=G
        % 'scale',S        Subsample the image by S in both directions.
        % 'resolution',S   Obtain an image of size S=[W H].
        % 'id',I           ID of camera
        %
        % Notes:
        % - The specified 'resolution' must match one that the camera is capable of,
        %   otherwise the result is not predictable.

            % invoke the superclass constructor and process common arguments
            m = m@ImageSource(varargin{:});
            
            % open the video source

            if length(varargin) > 0
                camera = -1;        % choose default
            else
                camera = args(1);
            end
            framegrabber(VideoCamera_fg.CAMERA_OP_OPEN, camera);
            
            for i=1:10
                k = framegrabber(VideoCamera_fg.CAMERA_OP_ISRUNNING);

                if k > 0
                    break;
                end
                pause(0.5);
            end


            % get the parameters of the video source
            sz = framegrabber(VideoCamera_fg.CAMERA_OP_GET_PARAMS);
            m.width = sz(1);
            m.height = sz(2);
            m.name = framegrabber(VideoCamera_fg.CAMERA_OP_GET_NAME);
        end
        
        function paramSet(v, a1)
        end

        % destructor
        function delete(m)
            framegrabber(VideoCamera_fg.CAMERA_OP_CLOSE);
        end

        function close(m)
        %VideoCamera_fg.close Close the image source
        %
        % V.close() closes the connection to the camera.
            framegrabber(VideoCamera_fg.CAMERA_OP_STOP);
            pause(2);
            framegrabber(VideoCamera_fg.CAMERA_OP_CLOSE);
        end

        function out = grab(m, opt)
        %VideoCamera_fg.grab Acquire image from the camera
        %
        % IM = V.grab() acquires an image from the camera.
        %
        % Notes::
        % - the function will block until the next frame is acquired.

            im = framegrabber(VideoCamera_fg.CAMERA_OP_GRAB);

            % apply options specified at construction time
            im = m.convert(im);
            m.width = numcols(im);
            m.height = numrows(im);
            
            if nargout > 0
                out = im;
            else
                idisp(im);
            end
        end

        function s = char(m)
        %VideoCamera_fg.char Convert to string
        %
        % V.char() is a string representing the state of the camera object in 
        % human readable form.
            s = '';

            s = strvcat(s, sprintf('VideoCamera. %s %d x %d', m.name, m.width, m.height));

            % show constructor time options
            s2 = char@ImageSource(m);
            if ~isempty(s2)
                s = strvcat(s, strcat(' - ', s2));
            end
        end

    end
end
