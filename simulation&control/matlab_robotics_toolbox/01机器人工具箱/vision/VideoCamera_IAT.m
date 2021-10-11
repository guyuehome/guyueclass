%VideoCamera_IAT Class to read from local video camera
%
% A concrete subclass of ImageSource that acquires images from a local
% camera using the MATLAB Image Acquisition Toolbox (imaq).  This Toolbox
% provides a multiplatform interface to a range of cameras, and this
% class provides a simple wrapper.
%
% This class is not intended to be used directly, instead use the factory
% method Video which will return an instance of this class if the Image
% Acquisition Toolbox is installed, for example
%
%         vid = VideoCamera();
%
% Methods::
% grab    Aquire and return the next image
% size    Size of image
% close   Close the image source
% char    Convert the object parameters to human readable string
%
% See also VideoCamera, ImageSource, AxisWebCamera, Movie.


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

classdef VideoCamera_IAT < ImageSource

    properties

        video
        adaptor
        continuous
    end

    methods(Static)
        % class method to list the available video sources

        function list()
            % list available adaptors and cameras

            hwinfo = imaqhwinfo();
            adaptors = hwinfo.InstalledAdaptors
            for adaptorName=adaptors
                adaptor = imaqhwinfo(adaptorName{1});
                fprintf('Adaptor: %s\n', adaptor.AdaptorName);
                for i=1:numel(adaptor.DeviceInfo)
                    info = adaptor.DeviceInfo(i);
                    fprintf('  %s (id=%d)\n', info.DeviceName, adaptor.DeviceIDs{i});
                    for format=info.SupportedFormats
                        fprintf('    %s', format{1});
                        if strcmp(format{1}, info.DefaultFormat)
                            fprintf(' (default)\n');
                        else
                            fprintf('\n');
                        end
                    end
                end
            end
        end
    end

    methods

        function m = VideoCamera_IAT(varargin)
        %VideoCamera_IAT.VideoCamera_IAT Video camera constructor
        %   
        % V = Video_IAT(CAMERA, OPTIONS) is a Video object that acquires
        % images from the local video camera specified by the string CAMERA.
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
            m = m@ImageSource({});
            m.video = [];
            m.adaptor = [];
            
            opt.continuous = [];
            nargin
            varargin
            [opt,args] = tb_optparse(opt, varargin);
            m.continuous = opt.continuous;
            if exist('imaqhwinfo')
                fprintf('Image acquisition toolbox detected\n');


                %res = regexp(a, '[0-9]+x[0-9]+', 'match')
                %x=sscanf(res{1}, '%dx%d')

                if length(args) == 0
                    % no adaptor given, scan for first one with devices
                    hwinfo = imaqhwinfo();
                    adaptors = hwinfo.InstalledAdaptors;
                    for adaptorName=adaptors
                        adaptor = imaqhwinfo(adaptorName{1});
                        if numel(adaptor.DeviceInfo) == 0
                            continue;
                        end
                    end
                    fprintf('Using adaptor %s\n', adaptor.AdaptorName);
                    m.video = videoinput(adaptor.AdaptorName);
                    m.adaptor = adaptor;
                elseif  length(args) == 1
                    % we were given an adaptor

                    if isempty(im.id)
                        im.id = 1;
                    end
                    m.vid = videoinput(camera, im.id);

                    fprintf('  %s (id=%d)\n', info.DeviceName, adaptor.DeviceIDs{i});
                    for format=info.SupportedFormats
                        fprintf('    %s', format);
                        if strcmp(format, info.DefaultFormat)
                            fprintf(' (default)\n');
                        else
                            fprintf('\n');
                        end
                    end
                end
            else
                error('no camera interface available');
            end
            set(m.video, 'ReturnedColorSpace', 'RGB');
            sz = get(m.video, 'VideoResolution');
            m.width = sz(1);
            m.height = sz(2);
            if m.continuous > 0
                set(m.video, 'FrameGrabInterval', m.continuous);
                start(m.video);
            end
        end
        
        function paramSet(v, a1)
        end

        % destructor
        function delete(m)
        end

        function close(m)
        %VideoCamera_IAT.close Close the image source
        %
        % V.close() closes the connection to the camera.

        end

        function [im, time] = grab(m, opt)
        %VideoCamera_IAT.grab Acquire image from the camera
        %
        % IM = V.grab() acquires an image from the camera.
        %
        % Notes::
        % - the function will block until the next frame is acquired.

            if m.continuous
                im = getdata(m.video);
            else
                im = getsnapshot(m.video);
            end
        end

        function preview(m, control)
            %VideoCamera_IAT.preview Control image preview
            %
            % V.preview(true) enables camera preview in a separate window
            %
            if control
                preview(m.video);
            else
                closepreview(m.video);
            end
        end

        function s = char(m)
        %VideoCamera_IAT.char Convert to string
        %
        % V.char() is a string representing the state of the camera object in 
        % human readable form.
            s = '';

            if ~isempty(m.adaptor)
                if m.continuous > 0
                    mode = sprintf('(continuous, every %d frame)', m.continuous);
                else
                    mode = '';
                end
                s = strvcat(s, sprintf('Video: %s%s %d x %d', ...
                    m.adaptor.AdaptorName, mode, m.width, m.height));

                % show constructor time options
                s2 = char@ImageSource(m);
                if ~isempty(s2)
                    s = strvcat(s, strcat(' - ', s2));
                end
            end
        end

    end
end
