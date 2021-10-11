%PICKREGION Pick a rectangular region of a figure using mouse
%
% [p1,p2] = PICKREGION() initiates a rubberband box at the current click point
% and animates it so long as the mouse button remains down.  Returns the first
% and last coordinates in axis units.
%
% Options::
% 'axis',A     The axis to select from (default current axis)
% 'ls',LS      Line style for foreground line (default ':y');
% 'bg'LS,      Line style for background line (default '-k');
% 'width',W    Line width (default 2)
%
% Notes::
% - Effectively a replacement for the builtin rbbox function which draws the box in
%   the wrong location on my Mac's external monitor.
%
% Author::
% Based on rubberband box from MATLAB Central  written/Edited by Bob Hamans 
% (B.C.Hamans@student.tue.nl) 02-04-2003, in turn based on an idea of 
% Sandra Martinka's Rubberline.
    
function [p1,p2]=pickregion(varargin)
    % handle options
    opt.axis = gca;
    opt.ls = ':y';
    opt.bg = '-k';
    opt.width = 2;
    
    opt = tb_optparse(opt, varargin);

    h = opt.axis;
    
    % Get current user data
    cudata=get(gcf,'UserData');
    hold on;
    
    % Wait for left mouse button to be pressed
    k=waitforbuttonpress;

    % get current point
    p1=get(h,'CurrentPoint');       %get starting point
    p1=p1(1,1:2);                   %extract x and y
    
    % create 2 overlaid lines for contrast:
    %   black solid
    %   color dotted
    lh1 = plot(p1(1),p1(2),opt.bg, 'LineWidth', opt.width);      %plot starting point
    lh2 = plot(p1(1), p1(2), opt.ls, 'LineWidth', opt.width);
    
    % Save current point and handles in user data
    udata.p1=p1;
    udata.h=h;
    udata.lh1=lh1;
    udata.lh2=lh2;
    
    % Set handlers for mouse up and mouse motion
    udata.wbupOld = get(gcf, 'WindowButtonUp');
    udata.wbmfOld = get(gcf, 'WindowButtonMotionFcn');
    set(gcf, ...
        'WindowButtonMotionFcn', @(src,event) wbmf(src,udata), ...
        'WindowButtonUp', @(src,event) wbup(src,udata), ...
        'DoubleBuffer','on');

    % Wait until the lines have been destroyed
    waitfor(lh1);
    
    % Get data for the end point
    p2=get(h,'Currentpoint');       %get end point
    p2=p2(1,1:2);                   %extract x and y

    % Remove the mouse event handlers and restore user data
    set(gcf,'UserData',cudata,'DoubleBuffer','off');
end

function wbmf(src, ud) %window motion callback function
    
    % get current coordinates
    P = get(ud.h,'CurrentPoint');
    P = P(1,1:2);
    
    % Use 5 point to draw a rectangular rubberband box
    xdata = [P(1),P(1),ud.p1(1),ud.p1(1),P(1)];
    ydata = [P(2),ud.p1(2),ud.p1(2),P(2),P(2)];
    
    % draw the two lines
    set(ud.lh1,'XData', xdata,'YData', ydata);
    set(ud.lh2,'XData', xdata,'YData', ydata);

end

function wbup(src, ud)
    % remove motion handler
    set(gcf, 'WindowButtonMotionFcn', ud.wbmfOld);
    set(gcf, 'WindowButtonUpFcn', ud.wbupOld);
    
    
    % delete the lines
    delete(ud.lh2);
    delete(ud.lh1);
end
