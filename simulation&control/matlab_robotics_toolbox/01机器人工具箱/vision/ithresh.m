%ITHRESH Interactive image threshold
%
% ITHRESH(IM) displays the image IM in a window with a slider which
% adjusts the binary threshold.
%
% ITHRESH(IM, T) as above but the initial threshold is set to T.
%
% IM2 = ITHRESH(IM) as above but returns the thresholded image after the
% "done" button in the GUI is pressed.
%
% [IM2,T] = ITHRESH(IM) as above but also returns the threshold value.
%
% Notes::
% - Greyscale image only.
% - For a uint8 class image the slider range is 0 to 255.
% - For a floating point class image the slider range is 0 to 1.0
% - The GUI only displays the "done" button if output arguments are
%   requested, otherwise the threshold window operates independently.
%
% See also IDISP.


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

function [imt, thresh] = ithresh(im, t)

    % number of colors
    ncolors = 256;
    true_high = true;

    if iscolor(im)
        error('for greyscale images only');
    end

    % set default threshold to 50%
    if nargin < 2
        t = 0.5;
    else
        if ~isfloat(im)
            t = t / cast(intmax(class(im)), 'double');
        end
    end

    clf
    figure(gcf);    % bring to top

    % set up the color map with defined limits
    set(gca, 'CLimMode', 'Manual');


    % display the image in a small set of axes
    h = image(im);
    set(h, 'CDataMapping', 'scaled');
    
    set(gca, 'Units', 'Normalized', ...
        'Position', [0.13 0.15 0.775 0.75]);
    if isfloat(im)
        set(gca, 'CLim', [0 1]);
    else
        set(gca, 'CLim', [0 intmax(class(im))]);
    end
    
    % create threshold display window
    htf = uicontrol(gcf, ...
            'style', 'text', ...
            'units',  'norm', ...
            'pos', [.75 .935 .2 .05], ...
            'background', [1 1 1], ...
            'HorizontalAlignment', 'left', ...
            'string', num2str(t) ...
        );

    
    if nargout > 0
        % create quit button if outputs required
        donebutton = uicontrol(gcf, ...
            'style', 'pushbutton', ...
            'units',  'norm', ...
            'pos', [.02 .935 .1 .05], ...
            'callback', 'uiresume', ...
            'string', 'done' ...
            );
    end
    
    % create user data structure
    %  ud.n           the number of colors in the color map
    %  ud.htf handle  for the threshold display
    %  ud. true_high  true if above threshold is shown white
    %  ud.max         the maximum pixel class value, Inf for float
    ud.n = ncolors;
    ud.htf = htf;
    ud.true_high = true_high;
    if isfloat(im)
        ud.max = Inf;
    else
        ud.max = intmax(class(im));
    end

    % create slider
    slider = uicontrol(gcf,'Style','Slider', ...
        'Units','norm', ...
        'Position',[0.04 0 0.9 .07], ...
        'Min', 0.0, ...
        'Max', 1.0, ...
        'UserData', ud, ...
        'Value', t, ...
        'Tag', 'threshold', ...
        'Callback', @thresh_callback );
    
    set_threshold(t, ud);

    %{
    THIS CODE BLOCK WORKS FINE IN SERIALLINK.TEACH BUT NOT HERE...
    % if findjobj exists use it, since it lets us get continous callbacks while
    % a slider moves
    if (exist('findjobj', 'file')>0) && ~ispc
        disp('using findjobj');
        drawnow
        jh = findjobj(slider, 'nomenu')
        jh.AdjustmentValueChangedCallback = @(src,event)thresh_callback(src, event);
    end
    %}
    
    set(gcf, 'name', 'ithresh');
    
    if nargout > 0
        % wait till the done button is pushed
        uiwait();
        
        % then return arguments as requested
        t = get(slider, 'Value');
        if ~isinf(ud.max)
            t = t * ud.max;
        end
        if nargout >= 1
            imt = im > t;
        end
        if nargout == 2
            thresh = t;
        end
    end
end

% invoked on a GUI event
function thresh_callback(obj, events)

    % get slider value
    t = get(obj, 'Value');

    ud = get(obj, 'UserData');
    set_threshold(t, ud);
end

function set_threshold(t, ud)
    % threshold is changed by manipulating the color map

    % round the threshold up to an integer color map index
    v = floor(t*(ud.n-1)+1);

    % create the color map
    if ud.true_high
        % high is white
        cmap = ones(ud.n,3);
        cmap(1:v,:) = 0;
    else
        % high is black
        cmap = zeros(ud.n,3);
        cmap(1:v,:) = 1;
    end
    colormap(cmap);

    % update the display window
    if isinf(ud.max)
        set(ud.htf, 'String', num2str(t));
    else
        set(ud.htf, 'String', num2str(t*ud.max));
    end

end
