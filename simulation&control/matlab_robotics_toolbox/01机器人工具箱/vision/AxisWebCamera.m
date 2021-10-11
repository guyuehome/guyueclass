%AXISWEBCAMERA Image from Axis webcam
%
% A concrete subclass of ImageSource that acquires images from a web camera
% built by Axis Communications (www.axis.com).
%
% Methods::
% grab    Aquire and return the next image
% size    Size of image
% close   Close the image source
% char    Convert the object parameters to human readable string
%
% See also ImageSource, Video.

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

classdef AxisWebCamera < ImageSource

    properties
        url
        firstImage
    end

    methods

        function wc = AxisWebCamera(url, varargin)
        %AxisWebCamera.AxisWebCamera Axis web camera constructor
        %
        % A = AxisWebCamera(URL, OPTIONS) is an AxisWebCamera object that acquires
        % images from an Axis Communications (www.axis.com) web camera.
        %
        % Options::
        % 'uint8'          Return image with uint8 pixels (default)
        % 'float'          Return image with float pixels
        % 'double'         Return image with double precision pixels
        % 'grey'           Return greyscale image
        % 'gamma',G        Apply gamma correction with gamma=G
        % 'scale',S        Subsample the image by S in both directions.
        % 'resolution',S   Obtain an image of size S=[W H].
        %
        % Notes:
        % - The specified 'resolution' must match one that the camera is capable of,
        %   otherwise the result is not predictable.

            % invoke the superclass constructor and process common arguments
            wc = wc@ImageSource(varargin{:});

            % set default size params if not set
            if isempty(wc.width)
                wc.width = 640;
                wc.height = 480;
            end

            wc.url = url;
            wc.firstImage = [];

            try
                wc.firstImage = wc.grab();
            except
                error('cant access specified web cam')
            end
            [height,width] = size(wc.firstImage);
            wc.color = ndims(wc.firstImage) > 2;
        end

        % handle parameters not known to the superclass
        function n = paramSet(wc, args)
            if nargin > 1
                switch lower(args{1})
                case 'resolution'
                    res = args{2};
                    res
                    wc.width = res(1);
                    wc.height = res(2);
                    n = 1;
                otherwise
                    error( sprintf('unknown option <%s>', args{count}));
                end
            end
        end

        function close(m)
        %AxisWebCamera.close Close the image source
        %
        % A.close() closes the connection to the web camera.
        end

        function im = grab(wc)
        %AxisWebCamera.grab Acquire image from the camera
        %
        % IM = A.grab() is an image acquired from the web camera.
        %
        % Notes::
        % - Some web cameras have a fixed picture taking interval, and this function
        %   will return the most recently captured image held in the camera.


            if ~isempty(wc.firstImage)
                % on the first grab, return the image we used to test the webcam at
                % instance creation time
                im = wc.convert( wc.firstImage );
                wc.firstImage = [];
            else

                url = sprintf('%s/axis-cgi/jpg/image.cgi?resolution=%dx%d', wc.url, wc.width, wc.height);
                im = wc.convert( imread(url) );
            end
            
            if nargout > 0
                out = im;
            else
                idisp(im);
            end

        end

        function s = char(wc)
        %AxisWebCamera.char Convert to string
        %
        % A.char() is a string representing the state of the camera object in 
        % human readable form.
        %
        % See also AxisWebCamera.display.

            s = '';
            s = strvcat(s, sprintf('Webcam @ %s', wc.url));
            if wc.iscolor()
                s = strvcat(s, sprintf('  %d x %d x 3', wc.width, wc.height));
            else
                s = strvcat(s, sprintf('  %d x %d', wc.width, wc.height));
            end
        end

    end
end
