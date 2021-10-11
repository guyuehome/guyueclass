%VideoCamera Abstract class to read from local video camera
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
function vid = VideoCamera(varargin)

    % this function looks like a class, the only way to implement the
    % factory design pattern

    if exist('imaqhwinfo', 'file')
        % we have the Mathworks Image Acquisition Toolbox
        if nargin == 1 && strcmp(varargin{1}, '?')
            VideoCamera_IAT.list();
        else
            vid = VideoCamera_IAT(varargin{:});
        end
    elseif exist('framegrabber', 'file') == 3
        % we have the MVTB framegrabber MEX interface, for either
        % MacOS, Linux or Windows
        if nargin == 1 && strcmp(varargin{1}, '?')
            VideoCamera_fg.list();
        else
            vid = VideoCamera_fg(varargin{:});
        end
    else
        error('no video capture capability on this computer');
    end
