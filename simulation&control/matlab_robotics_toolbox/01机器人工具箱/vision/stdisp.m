%STDISP Display stereo pair
%
% STDISP(L, R) displays the stereo image pair L and R in adjacent windows.
%
% Two cross-hairs are created.  Clicking a point in the left image positions
% black cross hair at the same pixel coordinate in the right image.  Clicking
% the corresponding world point in the right image sets the green crosshair
% and displays the disparity [pixels].
%
% See also IDISP, ISTEREO.


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

function stdisp(L, R)

    % display the images side by side
    idisp([L R], 'nogui');
    
    % initial cross hair location
    Y = 100;
    X = 100;

    % create the 3 lines segments and stash in user data on the axis
    ud.w = size(L,2);   % width of left image
    ud.hline = line('XData',get(gca,'XLim'),'YData',[Y Y], ...
                 'Tag','Horizontal Cursor');
    ud.vline_l = line('XData',[X X],'YData',get(gca,'YLim'), ...
                 'Tag','Vertical Cursor');
    ud.vline_r = line('XData',[X+ud.w X+ud.w],'YData',get(gca,'YLim'), ...
                 'Tag','Vertical Cursor');
    ud.vline_r2 = line('XData',[X+ud.w X+ud.w],'YData',get(gca,'YLim'), ...
                 'Tag','Vertical Cursor', 'color', 'g');
    ud.panel = uicontrol(gcf, ...
            'style', 'text', ...
            'units',  'norm', ...
            'pos', [.5 .935 .48 .05], ...
            'background', [1 1 1], ...
            'HorizontalAlignment', 'left', ...
            'string', ' Machine Vision Toolbox for Matlab  ' ...
        );
   set(gca, 'UserData', ud);

    % Set the WindowButtonFcn of the figure
    set(gcf,'WindowButtonDownFcn', @buttonDown,...
            'WindowButtonUpFcn',@buttonUp);	
                
end
        
function moveCursor(src, event)
    ud = get(gca, 'UserData');
    cp = get(gca,'CurrentPoint');
    % cp = [xfront yfront xfront; xback yback zback]

    if cp(1,1) < ud.w
        % clicked in the left pane
        set(ud.hline, 'YData', [cp(1,2) cp(1,2)]);
        set(ud.vline_l, 'XData', [cp(1,1) cp(1,1)]);
        set(ud.vline_r, 'XData', ud.w+[cp(1,1) cp(1,1)]);
    else
        % clicked in the right pane
        set(ud.vline_r2, 'XData', [cp(1,1) cp(1,1)]);
        xl = get(ud.vline_l, 'XData');
        yl = get(ud.hline, 'YData');
        %fprintf('d = %f\n', cp(1,1) - xl(1) - ud.w);
        set(ud.panel, 'string', sprintf('dh = %.2f, dv = %.2f\n',  ...
            xl(1) + ud.w - cp(1,1), cp(1,2) - yl(1)));
    end
end

function buttonDown(src, event)
    set(gcf, 'WindowButtonMotionFcn',@moveCursor);
    moveCursor(src, event);
    %disp('down');
end

function buttonUp(src, event)
    %disp('up');
    set(gcf, 'WindowButtonMotionFcn','');
end
