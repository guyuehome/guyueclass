%ImageSource Abstract class for image sources
%
% An abstract superclass for implementing image sources.
%
% Methods::
% grab       Aquire and return the next image
% close      Close the image source
% iscolor    True if image is color
% size       Size of image
% char       Convert image source parameters to human readable string
% display    Display image source parameters in human readable form
%
% See also AxisWebCamera, Video, Movie.


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

classdef ImageSource < handle

    properties
        width           % width of each frame
        height          % height of each frame
        color           % is a color image

        % options set at construction time
        imageType
        makeGrey
        gamma
        scaleFactor
    end

    methods (Abstract)
        im = grab()
        close()
        paramSet()
    end

    methods

        function imsource = ImageSource(varargin)
        %ImageSource.ImageSource Image source constructor
        %
        % I = ImageSource(OPTIONS) is an ImageSource object that holds parameters
        % related to acquisition from some particular image source.
        %
        % Options::
        % 'width',W    Set image width to W
        % 'height',H   Set image height to H
        % 'uint8'      Return image with uint8 pixels (default)
        % 'int16'      Return image with int16 pixels
        % 'int32'      Return image with int32 pixels
        % 'float'      Return image with float pixels
        % 'double'     Return image with double precision pixels
        % 'grey'       Return image is greyscale
        % 'gamma',G    Apply gamma correction with gamma=G
        % 'scale',S    Subsample the image by S in both directions.

            % set default options
            opt.imageType = {'uint8', 'float', 'double'};
            opt.grey = false;
            opt.gamma = [];
            opt.scale = 1;
            opt.width = [];
            opt.height = [];

            [opt,args] = tb_optparse(opt, varargin);
            
            imsource.imageType = opt.imageType;
            imsource.makeGrey = opt.grey;
            imsource.gamma = opt.gamma;
            imsource.scaleFactor = opt.scale;
            imsource.width = opt.width;
            imsource.height = opt.height;
            

            % remaining arguments get passed to the subclass
            imsource.paramSet(args{:});
    
        end

        function b = iscolor(imsource)
            b = imsource.color;
        end

        function im2 = convert(imsource, im)

            im2 = [];
            % apply options specified at construction time
            if imsource.scaleFactor > 1
                im = im(1:imsource.scaleFactor:end, 1:imsource.scaleFactor:end, :);
            end
            if imsource.makeGrey & (ndims(im) == 3)
                im = imono(im);
            end
            if ~isempty(imsource.imageType)
                switch imsource.imageType
                case 'double'
                    im = idouble(im);
                case 'float'
                    im = idouble(im, 'float');
                otherwise
                    im = cast(im, imsource.imageType);
                end
            end

            if ~isempty(imsource.gamma)
                im = igamma(im, imsource.gamma);
            end

            if isempty(im2)
                im2 = im;
            end
        end

        function s = size(imsource)
            s = [imsource.height imsource.width];
        end
            
        function s = char(imsource)
            s = '';
            if imsource.scaleFactor > 1
                s = strcat(s, sprintf('subsample by %d: ', imsource.scaleFactor));
            end
            if imsource.makeGrey
                s = strcat(s, 'convert to grey: ');
            end
            if ~isempty(imsource.imageType)
                s = strcat(s, sprintf('cast to %s: ', imsource.imageType));
            end
            if ~isempty(imsource.gamma)
                s = strcat(s, sprintf('gamma correct %f: ', imsource.gamma));
            end
        end

        function display(imsource)
        %ImageSource.display Display value
        %
        % I.display() displays the state of the image source object in human
        % readable form.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is an ImageSource object and the command has no trailing
        %   semicolon.

            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(imsource))
            if loose
                disp(' ');
            end
        end
    end
end
