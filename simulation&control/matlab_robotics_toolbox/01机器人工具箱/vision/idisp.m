%IDISP	Interactive image display tool
%
% IDISP(IM, OPTIONS) displays an image and allows interactive investigation
% of pixel values, linear profiles, histograms and zooming.  The image is
% displayed in a figure with a toolbar across the top.  If IM is a cell
% array of images, they are first concatenated (horizontally).
%
% User interface::
%
% - Left clicking on a pixel will display its value in a box at the top.
% - The "line" button allows two points to be specified and a new figure
%   displays intensity along a line between those points.
% - The "histo" button displays a histogram of the pixel values in a new 
%   figure.  If the image is zoomed, the histogram is computed over only 
%   those pixels in view.
% - The "zoom" button requires a left-click and drag to specify a box 
%   which defines the zoomed view.
% - The "colormap" button is displayed only for greyscale images, and is
%   a popup button that allows different color maps to be selected.
%
% Options::
% 'nogui'          don't display the GUI
% 'noaxes'         don't display axes on the image
% 'noframe'        don't display axes or frame on the image
% 'plain'          don't display axes, frame or GUI
% 'axis',A         display the image in the axes given by handle A, the
%                  'nogui' option is enforced.
% 'here'           display the image in the current axes
% 'title',T        put the text T in the title bar of the window
% 'clickfunc',F    invoke the function handle F(x,y) on a down-click in
%                  the window
% 'ncolors',N      number of colors in the color map (default 256)
% 'bar'            add a color bar to the image
% 'print',F        write the image to file F in EPS format
% 'square'         display aspect ratio so that pixels are square
% 'wide'           make figure full screen width, useful for displaying stereo pair
% 'flatten'        display image planes (colors or sequence) as horizontally 
%                  adjacent images
% 'ynormal'        y-axis increases upward, image is inverted
% 'histeq'         apply histogram equalization
% 'cscale',C       C is a 2-vector that specifies the grey value range that
%                  spans the colormap.
% 'xydata',XY      XY is a cell array whose elements are vectors that span
%                  the x- and y-axes respectively.
% 'colormap',C     set the colormap to C (Nx3)
% 'grey'           color map: greyscale unsigned, zero is black, maximum
%                  value is white
% 'invert'         color map: greyscale unsigned, zero is white, maximum 
%                  value is black
% 'signed'         color map: greyscale signed, positive is blue, negative
%                  is red, zero is black
% 'invsigned'      color map: greyscale signed, positive is blue, negative
%                  is red, zero is white
% 'random'         color map: random values, highlights fine structure
% 'dark'           color map: greyscale unsigned, darker than 'grey', 
%                  good for superimposed graphics
% 'new'            create a new figure
%
% Notes::
% - Is a wrapper around the MATLAB builtin function IMAGE. See the MATLAB help
%   on "Display Bit-Mapped Images" for details of color mapping.
% - Color images are displayed in MATLAB true color mode: pixel triples map to 
%   display RGB values.  (0,0,0) is black, (1,1,1) is white.
% - Greyscale images are displayed in indexed mode: the image pixel value is 
%   mapped through the color map to determine the display pixel value.
% - For grey scale images the minimum and maximum image values are mapped to 
%   the first and last element of the color map, which by default ('greyscale')
%   is the range black to white. To set your own scaling between displayed 
%   grey level and pixel value use the 'cscale' option.
% - The title of the figure window by default is the name of the variable
%   passed in as the image, this can't work if the first argument is an expression.
%
% Examples::
%   Display 2 images side by side
%        idisp({im1, im2})
%
%   Display image in a subplot
%        subplot(211)
%        idisp(im, 'axis', gca);
%
%   Call a user function when you click a pixel
%        idisp(im, 'clickfunc', @(x,y) fprintf('hello %d %d\n', x,y))
%
%   Set a colormap, in this case a MATLAB builtin one
%        idisp(im, 'colormap', cool);
%
%   Display an image which contains a map of a region, perhaps an obstacle grid,
%   that spans real world dimensions x, y in the range -10 to 10.
%        idisp(map, 'xyscale', {[-10 10], [-10 10]});
%
% See also IMAGE, CAXIS, COLORMAP, ICONCAT.



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
%  put operations off a menubutton
%  for zooming show the window within the big big picture, use an inset
%  image

function idisp(im, varargin)

%TODO
% display Inf/NaN as red

    if ~(isnumeric(im) || islogical(im) || iscell(im))
        error('expecting an image (2D or 3D matrix)');
    end

    %---- process the options
    opt.ncolors = 256;
    opt.gui = true;
    opt.axes = true;
    opt.square = true;
    opt.wide = false;
    opt.colormap = [];
    opt.print = [];
    opt.bar = false;
    opt.frame = true;
    opt.ynormal = false;
    opt.cscale = [];
    opt.xydata = [];
    opt.plain = false;
    opt.flatten = false;
    opt.toolbar = true;
    opt.title = [];
    opt.clickfunc = [];
    opt.colormap_std = {[], 'grey', 'signed', 'invsigned', 'random', 'invert', 'dark'};
    opt.axis = [];
    opt.histeq = false;
    opt.here = false;
    opt.new = false;
    opt.figure = [];
    
    [opt,arglist] = tb_optparse(opt, varargin);
    
    ud.ncolors = opt.ncolors;
    ud.clickfunc = opt.clickfunc;

    if opt.new
        figure
    end
    if ~isempty(opt.figure)
        figure(opt.figure);
    end

    if opt.plain
        opt.frame = false;
        opt.gui = false;
    end
    if ~isempty(opt.print)
        opt.gui = false;
    end
    if opt.flatten
        if size(im, 4) > 1
            im = permute(im, [1 2 4 3]);
        end
            im = reshape( im, size(im,1), size(im,2)*size(im,3), size(im,4) );
    end
    if ~isempty(opt.axis)
        opt.gui = false;
    end
    if strcmp( get(gcf,'WindowStyle'), 'docked')
        %opt.gui = false;
    end
    
    if length(arglist) ~= 0
        warning(['Unknown options: ', arglist]);
    end

    if opt.here
        opt.axis = gca;
        opt.gui = false;
    end

    %---- display the image
    if isempty(opt.axis)
        clf
    else
        axes(opt.axis)
    end
    
    if iscell(im)
        % image is a cell array
        [im,ud.u0] = iconcat(im);
    end

    ud.size = size(im);
    
    if opt.histeq
        im = inormhist(im);
    end
    
    % get the min/max values to do some color map scaling
    set(gca, 'CLimMode', 'Manual');

   
    % find min/max values and handle the case they are equal
    if isfloat(im)
        i_min = min(im(:));
        i_max = max(im(:));
        if abs(i_min - i_max) < eps
            i_min = 0;
            i_max = 1;
        end
    elseif isinteger(im)
        i_min = min(im(:));
        i_max = max(im(:));
        if i_min == i_max
            i_min = 0;
            i_max = intmax(class(im));
        end
    elseif islogical(im)
        % for logical images, don't do min/max test
        i_min = 0;
        i_max = 1;
    end
    set(gca, 'CLim', [i_min, i_max]);
    
    if ~isempty(opt.xydata)
        ud.image = image(opt.xydata{1}, opt.xydata{2}, im, 'CDataMapping', 'scaled');
    else
        ud.image = image(im, 'CDataMapping', 'scaled');
    end

    if opt.wide
        set(gcf, 'units', 'norm');
        pos = get(gcf, 'pos');
        set(gcf, 'pos', [0.0 pos(2) 1.0 pos(4)]);
    end

    %---- choose and set a color map
    if isempty(opt.colormap_std)
        if isempty(opt.colormap)
            % default colormap
            %disp('default color map');
            cmap = gray(opt.ncolors);
        else
            % load a Matlab color map
            cmap = opt.colormap;
        end
    else
        % a builtin shorthand color map was specified
        %disp(['builtin color map: ', opt.colormap_std]);
        cmap = custom_colormap(opt.colormap_std, opt.ncolors);
        
        mn = min(im(:));
        mx = max(im(:));
        set(gca, 'CLimMode', 'Manual');
        if mn < 0 && mx > 0
            a = max(-mn, mx);
            set(gca, 'CLim', [-a a]);
        elseif mn > 0
            set(gca, 'CLim', [-mx mx]);
        elseif mx < 0
            set(gca, 'CLim', [-mn mn]);
        end
    end
    colormap(cmap);

    %---- handle various display options
    if opt.bar
        colorbar
    end
    if opt.frame
        if opt.axes
            xlabel('u (pixels)');
            ylabel('v (pixels)');
        else
            set(gca, 'Xtick', [], 'Ytick', []);
        end
    else
        set(gca, 'Visible', 'off');
    end
    if opt.square
        set(gca, 'DataAspectRatio', [1 1 1]);
    end
    if opt.ynormal
        set(gca, 'YDir', 'normal');
    end
    set(ud.image, 'CDataMapping', 'scaled');
    if ~isempty(opt.cscale)
        set(gca, 'Clim', opt.cscale);
    end
    
    if isempty(opt.axis)
        figure(gcf);    % bring to top
    end

    if opt.print
        print(opt.print, '-depsc');
        return
    end
    
    % label the figure
    if isempty(opt.title)
        % show the variable name in the figure's title bar
        varname = inputname(1);
        if isempty(varname)
            set(gcf, 'name', 'idisp');
        else
            set(gcf, 'name', sprintf('idisp: %s', varname));
        end
    else
        set(gcf, 'name', opt.title);
    end
    ud.fig = gcf;
    
    %--- display the idisp tool bar
    if opt.gui
        if ~opt.toolbar
            set(gcf, 'MenuBar', 'none');
            set(gcf, 'ToolBar', 'none');
        end
        
        % create the panel itself
        bgcol = [135 206 250]/255;  % background color
        
        panel = uipanel(gcf, ...
            'BackGroundColor', bgcol,...
            'Units', 'Normalized', ...
            'Position', [0 0.92 1 0.08]);
        set(panel, 'Units', 'pixels'); % stop automatic resizing
        
        
        ud.axis = gca;
        ud.panel = panel;
        set(gca, 'UserData', ud);
        set(gca, 'Units', 'Normalized', ...
            'OuterPosition', [0 0 1 0.92]);
        set(ud.image, 'UserData', ud);
        
        % create pushbuttons
        bw = 0.08;
        bs = 0.09;
        
        ud.text = uicontrol(panel, 'Style', 'text', ...
            'Units','Normalized', ...
            'Position', [0.60 0.2 0.38 0.6], ...
            'background', [1 1 1], ...
            'HorizontalAlignment', 'left', ...
            'string', ' Machine Vision Toolbox for MATLAB  ' ...
            );
        uicontrol(panel,'Style','Push', ...
            'String','line', ...
            'Foregroundcolor', [0 0 1], ...
            'Units','Normalized', ...
            'Position',[0 0.1 bw 0.8], ...
            'UserData', ud, ...
            'Callback', @(src,event) line_callback(ud, src) );
        uicontrol(panel,'Style','Push', ...
            'String','histo', ...
            'Foregroundcolor', [0 0 1], ...
            'Units','Normalized', ...
            'Position',[bs 0.1 bw 0.8], ...
            'UserData', ud, ...
            'Callback', @(src,event) histo_callback(ud, src) );
        uicontrol(panel,'Style','Push', ...
            'String','zoom', ...
            'Foregroundcolor', [0 0 1], ...
            'Units','Normalized', ...
            'Position', [2*bs 0.1 bw 0.8], ...
            'Userdata', ud, ...
            'Callback', @(src,event) zoom_callback(ud, src) );
        uicontrol(panel,'Style','Push', ...
            'String','unzoom', ...
            'Foregroundcolor', [0 0 1], ...
            'Units','Normalized', ...
            'Position', [3*bs 0.1 1.5*bw 0.8], ...
            'Userdata', ud, ...
            'Callback', @(src,event) unzoom_callback(ud, src) );
        
        if ndims(im) == 2
            % NOTE: the height of the popup is a function of font and cannot be set
            uicontrol(panel,'Style','Popup', ...
                'String','grey|signed|invsigned|random|invert|dark', ...
                'Foregroundcolor', [0 0 1], ...
                'Units','Normalized', ...
                'Position', [4.5*bs 0 2*bw 0.8], ...
                'Userdata', ud, ...
                'Callback', @(src,event) colormap_callback(ud, src) );
        end
        
        %set(gcf, 'Color', [0.2 0.2 0.2]*2, ...
        set(gcf, ...
            'WindowButtonDownFcn', @(src,event) button_down_callback(ud, src), ...
            'WindowButtonUpFcn', @(src,event) button_up_callback(ud, src), ...
            'ResizeFcn', @(src,event) resize_callback(panel));
        
        set(ud.image, 'DeleteFcn', @cleanup_callback);
        %set(hi, 'UserData', ud);
    end
end

function button_down_callback(ud, src)
    if ~isempty(ud)
        % install pixel value inspector
        set(ud.fig, 'WindowButtonMotionFcn', @(src,event) display_update(ud) );
        display_update(ud);
        
        if ~isempty(ud.clickfunc)
            cp = get(ud.axis, 'CurrentPoint');
            x = round(cp(1,1));
            y = round(cp(1,2));
            ud.clickfunc(x, y);
        end
    end
end

function display_update(ud)
    if ~isempty(ud)
        cp = get(ud.axis, 'CurrentPoint');
        x = round(cp(1,1));
        y = round(cp(1,2));
        try
            imdata = get(ud.image, 'CData');
            set(ud.text, 'String', sprintf(' (%d, %d) = %s', x, y, num2str(imdata(y,x,:), 4)));
            drawnow
        end
    end
end

function button_up_callback(ud, src)
    set(ud.fig, 'WindowButtonMotionFcn', '');
end

function zoom_callback(ud, src)
    [p1, p2] = pickregion();
    cp0 = floor( p1 );
    cp1 = floor( p2 );
    
    % determine the bounds of the ROI
    top = cp0(1,2);
    left = cp0(1,1);
    bot = cp1(1,2);
    right = cp1(1,1);
    if bot<top,
        t = top;
        top = bot;
        bot = t;
    end
    if right<left,
        t = left;
        left = right;
        right = t;
    end
    
    % extract the view region
    axis(ud.axis, [left right top bot]);
end

function unzoom_callback(ud, src)
    axes(ud.axis);
    axis([1 ud.size(2) 1 ud.size(1)]);
end

function line_callback(ud, src)
    
    set(ud.text, 'String', 'Click first point');
    axes(ud.axis);
    [x1,y1] = ginput(1);
    x1 = round(x1); y1 = round(y1);
    set(ud.text, 'String', 'Click last point');
    [x2,y2] = ginput(1);
    x2 = round(x2); y2 = round(y2);
    set(ud.text, 'String', '');
    imdata = get(ud.image, 'CData');
    
    % draw a green line to show where the profile was taken
    hold on
    plot([x1 x2], [y1 y2], 'g');
    hold off
    dx = x2-x1; dy = y2-y1;
    if abs(dx) > abs(dy),
        x = x1:x2;
        y = round(dy/dx * (x-x1) + y1);
        figure
        
        if size(imdata,3) > 1
            set(gca, 'ColorOrder', eye(3,3), 'Nextplot', 'replacechildren');
            n = size(imdata,1)*size(imdata,2);
            z = [];
            for i=1:size(imdata,3)
                z = [z imdata(y+x*numrows(imdata)+(i-1)*n)'];
            end
            plot(z);
        else
            plot(imdata(y+x*numrows(imdata)))
        end
    else
        y = y1:y2;
        x = round(dx/dy * (y-y1) + x1);
        figure
        if size(imdata,3) > 1
            set(gca, 'ColorOrder', eye(3,3), 'Nextplot', 'replacechildren');
            n = size(imdata,1)*size(imdata,2);
            z = [];
            for i=1:size(imdata,3)
                z = [z imdata(y+x*numrows(imdata)+(i-1)*n)'];
            end
            plot(y');
        else
            plot(imdata(y+x*numrows(imdata)))
        end
        
    end
    title(sprintf('Pixel profile (%d,%d) to (%d,%d)', x1, y1, x2, y2));
    xlabel('distance (pixels)')
    ylabel('greyscale');
    grid on
end

function histo_callback(ud, src)   
    imdata = get(ud.image, 'CData');
    b = floor(axis);   % bounds of displayed image
    if b(1) == 0,
        b = [1 b(2) 1 b(4)];
    end
    
    figure
    imdata = double(imdata(b(3):b(4), b(1):b(2),:));
    ihist(imdata);
end

function colormap_callback(ud, src)
    i = get(src, 'Value');
    names = {'grey', 'signed', 'invsigned', 'random', 'invert', 'dark'};
    cmap = custom_colormap(names{i}, ud.ncolors);
    colormap(ud.axis, cmap);
end

function cmap = custom_colormap(name, n)
    if nargin < 2
        n = 256;
    end
    
    switch name
        case 'random'
            cmap = rand(n,3);
        case 'dark'
            cmap = gray(n)*0.5;
        case 'grey'
            cmap = gray(n);
        case 'invert'
            % invert the monochrome color map: black <-> white
            cmap = gray(n);
            cmap = cmap(end:-1:1,:);
        case {'signed', 'invsigned'}
            % signed color map, red is negative, blue is positive, zero is black
            % inverse signed color map, red is negative, blue is positive, zero is white
            cmap = zeros(n, 3);
            opt.ncolors = bitor(n, 1);    % ensure it's odd
            ncm2 = ceil(n/2);
            if strcmp(name, 'signed')
                % signed color map, red is negative, blue is positive, zero is black
                for i=1:n
                    if i > ncm2
                        cmap(i,:) = [0 0 1] * (i-ncm2) / ncm2;
                    else
                        cmap(i,:) = [1 0 0] * (ncm2-i) / ncm2;
                    end
                end
            else
                % inverse signed color map, red is negative, blue is positive, zero is white
                for i=1:n
                    if i > ncm2
                        s = (i-ncm2)/ncm2;
                        cmap(i,:) = [1-s 1-s 1];
                    else
                        s = (ncm2-i)/ncm2;
                        cmap(i,:) = [1 1-s 1-s];
                    end
                end
            end
        otherwise
            warning('MVTB:idisp:badval', 'illegal color map name %s', name);
    end
end

function resize_callback(panel)
    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(gca, 'Units', 'pixels', ...
        'OuterPosition', [0 0 fs(3) fs(4)-ps(4)]);
    % keep the panel anchored to the top left corner
    set(panel, 'Position', [0 fs(4)-ps(4) fs(3) ps(4)]);
end

function cleanup_callback(im, event)
    % come here on clf of the figure window, remove all handlers
    set(gcf, ...
        'WindowButtonDownFcn', [], ...
        'WindowButtonUpFcn', [], ...
        'ResizeFcn', []);

    set(im, 'DeleteFcn', []);
end
