%IREAD  Read image from file
%
% IM = IREAD() presents a file selection GUI from which the user can select
% an image file which is returned as a matrix.  On subsequent calls 
% the initial folder is as set on the last call.
%
% IM = IREAD([], OPTIONS) as above but allows options to be specified.
%
% IM = IREAD(PATH, OPTIONS) as above but the GUI is set to the folder specified
% by PATH.  If the path is not absolute it is searched for on the MATLAB search 
% path.
%
% IM = IREAD(FILE, OPTIONS) reads the specified image file and returns a matrix.
% If the path is not absolute it is searched for on MATLAB search path.
%
% The image can be greyscale or color in any of a wide range of formats 
% supported by the MATLAB IMREAD function.
%
% Wildcards are allowed in file names.  If multiple files match a 3D or 4D image
% is returned where the last dimension is the number of images in the sequence.
%
% Options::
% 'uint8'      return an image with 8-bit unsigned integer pixels in 
%              the range 0 to 255
% 'single'     return an image with single precision floating point pixels
%              in the range 0 to 1.
% 'double'     return an image with double precision floating point pixels
%              in the range 0 to 1.
% 'grey'       convert image to greyscale, if it's color, using ITU rec 601
% 'grey_709'   convert image to greyscale, if it's color, using ITU rec 709
% 'gamma',G    apply this gamma correction, either numeric or 'sRGB'
% 'reduce',R   decimate image by R in both dimensions
% 'roi',R      apply the region of interest R to each image, 
%              where R=[umin umax; vmin vmax].
%
% Examples::
%  Read a color image and display it
%         >> im = iread('lena.png');
%         >> about im
%         im [uint8] : 512x512x3 (786.4 kB)
%         >> idisp(im);
%
%  Read a greyscale image sequence
%         >> im = iread('seq/*.png');
%         >> about im
%         im [uint8] : 512x512x9 (2.4 MB)
%         >> ianimate(im, 'loop');
% Notes::
% - A greyscale image is returned as an HxW matrix
% - A color image is returned as an HxWx3 matrix
% - A greyscale image sequence is returned as an HxWxN matrix where N is the 
%   sequence length 
% - A color image sequence is returned as an HxWx3xN matrix where N is the 
%   sequence length 
%
% See also IDISP, IANIMATE, IMONO, IGAMMA, IMREAD, IMWRITE, PATH.



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


function [I,info] = iread(filename, varargin)
    persistent lastPath

    % options
    %
    %   'float
    %   'uint8
    %   'grey'
    %   'gray'
    %   'grey_601'
    %   'grey_709'
    %   'grey_value'
    %   'gray_601'
    %   'gray_709'
    %   'gray_value'
    %   'reduce', n

    opt.type = {[], 'double', 'single', 'uint8'};
    opt.mkGrey = {[], 'grey', 'gray', 'mono', '601', 'grey_601', 'grey_709'};
    opt.gamma = [];
    opt.reduce = 1;
    opt.roi = [];
    opt.disp = [];

    opt = tb_optparse(opt, varargin);
    im = [];
    
    fileFilter = {
            '*.gif;*.png;*.pgm;*.ppm;*.pnm;*.jpg;*.tif;*.GIF;*.PNG;*.PGM;*.PPM;*.PNM;*.JPG;*.TIF;', 'All images';
            '*.pgm;*.PGM', 'PGM images';
            '*.jpg;*.JPG', 'JPEG images';
            '*.pgm;*.ppm;*.pnm;*.PGM;*.PPM;*.PNM', 'PBMplus images';
            '*.gif;*.png;*.jpg;*.GIF;*.PNG;*.JPG', 'web images';
            '*.*', 'All files';
            };
    
    if nargin == 0 || isempty(filename)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %invoke file browser GUI
        
        if ~isempty(lastPath)
            [file, npath] = uigetfile(fileFilter, 'iread', lastPath);
        else
            [file, npath] = uigetfile(fileFilter, 'iread');
        end
        
        if file == 0
            fprintf('iread canceled from GUI\n');
            return; % cancel button pushed
        else
            % save the path away for next time
            lastPath = npath;
            filename = fullfile(lastPath, file);
            im = loadimg(filename, opt);
        end
    elseif exist(filename,'dir') == 7
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % invoke file browser GUI on specified path
        [file,npath] = uigetfile(fileFilter, 'iread', filename);
        if file == 0
            fprintf('iread canceled from GUI\n');
            return; % cancel button pushed
        else
            % save the path away for next time
            lastPath = npath;
            filename = fullfile(lastPath, file);
            im = loadimg(filename, opt);
        end
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % some kind of wildcard filespec has been given
        if ~isempty(strfind(filename, '*')) | ~isempty(strfind(filename, '?')),
            % wild card files, eg.  'seq/*.png', we need to look for a folder
            % seq somewhere along the path.
                        [pth,name,ext] = fileparts(filename);

            if opt.verbose
                fprintf('wildcard lookup: %s %s %s\n', pth, name, ext);
            end
            
            % search for the folder name along the path
            folderonpath = pth;
            if ~(isempty(pth) || pth(1) == '.' || pth(1) == '/')
                
                for p=path2cell(path)'  % was userpath
                    if exist( fullfile(p{1}, pth) ) == 7
                        folderonpath = fullfile(p{1}, pth);
                        if opt.verbose
                            fprintf('folder found\n');
                        end
                        break;
                    end
                end
            end
            s = dir( fullfile(folderonpath, [name, ext]));      % do a wildcard lookup

            if length(s) == 0
                error('no matching files found');
            end

            for i=1:length(s)
                im1 = loadimg( fullfile(folderonpath, s(i).name), opt);
                if i==1
                    % preallocate storage, much quicker
                    im = zeros([size(im1) length(s)], class(im1));
                end
                if ndims(im1) == 2
                    im(:,:,i) = im1;
                elseif ndims(im1) == 3
                    im(:,:,:,i) = im1;
                end
            end
        else
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % simple file, no wildcard
            if strncmp(filename, 'http://', 7)
                im = loadimg(filename, opt);
            elseif exist(filename)
                im = loadimg(filename, opt);
            else
                % see if it exists on the Matlab search path
                for p=path2cell(path)
                    fname = fullfile(p{1}, filename);
                    if exist( fname ) > 0
                        im = loadimg(fullfile(p{1}, filename), opt);
                        break;
                    end
                end
            end
        end
    end

                      
    if isempty(im)
        error(sprintf('can''t find/open file: %s', filename));
    end
    if nargout > 0
        I = im;
        if nargout > 1
            info = imfinfo(filename);
        end
    else
        % if no output arguments display the image
        if ndims(I) <= 3
            idisp(I);
        end
    end
end

% load image from file name and apply options
function im = loadimg(filename, opt)

    % now we read the image
    im = imread(filename);

    if opt.verbose
        if ndims(im) == 2
            fprintf('loaded %s, %dx%d\n', filename, size(im,2), size(im,1));
        elseif ndims(im) == 3
            fprintf('loaded %s, %dx%dx%d\n', filename, size(im,2), size(im,1), size(im,3));
        end
    end

    % optionally convert it to greyscale using specified method
    if ~isempty(opt.mkGrey) && (ndims(im) == 3)
        im = imono(im, opt.mkGrey);
    end

    % optionally chop out a roi
    if ~isempty(opt.roi)
        im = iroi(im, opt.roi);
    end

    % optionally decimate it
    if opt.reduce > 1
        im = idecimate(im, opt.reduce);
    end

    % optionally convert to specified numeric type
    if ~isempty(opt.type)
        if isempty(findstr(opt.type, 'int'))
            im = cast(im, opt.type) / double(intmax(class(im)));
        else
            im = cast(im, opt.type);
        end
    end

    % optionally gamma correct it
    if ~isempty(opt.gamma)
        im = igamm(im, opt.gamma);
    end

    if opt.disp
        idisp(im);
    end

end

function c = path2cell(s)
    remain = s;
    c = {};
    while true
        [str, remain] = strtok(remain, ':');
        if isempty(str), break; end
        c = [c str];
    end
end
