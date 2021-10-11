%MPLOT	Plot time series data
%
% MPLOT(Y, OPTIONS) plots the time series data Y(NxM) in multiple 
% subplots.  The first column is assumed to be time, so M-1 plots are
% produced.
%
% MPLOT(T, Y, OPTIONS) plots the time series data Y(NxM) in multiple 
% subplots.  Time is provided explicitly as the first argument so M plots
% are produced.
%
% MPLOT(S, OPTIONS) as above but S is a structure.  Each field is assumed 
% to be a time series which is plotted.  Time is taken from the field 
% called 't'.
%
% MPLOT(W, OPTIONS) as above but W is a structure created by the Simulink 
% write to workspace block where the save format is set to "Structure 
% with time". Each field in the signals substructure is plotted.
%
% MPLOT(R, OPTIONS) as above but R is a Simulink.SimulationOutput object 
% returned by the Simulink sim() function.
%
% Options::
% 'col',C     Select columns to plot, a boolean of length M-1 or a list of
%             column indices in the range 1 to M-1
% 'label',L   Label the axes according to the cell array of strings L
% 'date'      Add a datestamp in the top right corner
%
% Notes::
% - In all cases a simple GUI is created which is invoked by a right
%   click anywhere on one of the plots.  The supported options are:
%   - 
%

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function mplot(varargin)
    
    opt.label = [];
    opt.date = false;
    opt.cols = [];
    
    [opt,args] = tb_optparse(opt, varargin);
    
    if isstruct(args{1})
        s = args{1};
        if isfield(s, 'signals'),
            % To Workspace type structure
            matplot(s.time, s.signals.values, opt);
            if isfield(s, 'blockName'),
                title(s.blockName)
            end
        else
            % retriever type structure
            structplot(args{:})
        end
        
    elseif isa(args{1}, 'Simulink.SimulationOutput')
        % Simulink output object
        s = args{1};
        matplot(s.find('tout'), s.find('yout'), opt);
        if isfield(s, 'blockName'),
            title(s.blockName)
        end
    else
        matplot(args{:}, opt)
end
    
    if opt.date
        datestamp
    end
    
    if ~isempty(opt.label)
        mlabel(opt.label);
    end
    
    mtools
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function structplot(s)
    if ~isfield(s, 't') & ~isfield(s, 'time')
        error('structure must include a time element t')
    end
    if isfield(s, 't')
        t = s.t;
    elseif isfield(s, 'time'),
        t = s.time;
    end
    f = fieldnames(s);
    n = length(f) - 1;
    sp = n*100 + 11;
    tmax = max(t);
    i = 1;
    for ff = f'
        fieldnam = char(ff);
        switch fieldnam,
            case {'t', 'time'},
            otherwise,
                h(i) = subplot(sp);
                plot(t, getfield(s, fieldnam));
                set(h(i), 'UserData', i);
                v = axis;
                v(2) = tmax;
                axis(v);
                grid
                xlabel('Time');
                ylabel(fieldnam);
                sp = sp +1;
                i = i + 1;
        end
    end
    axes(h(1));
    figure(gcf)
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% legacy function for matrix input data, old mplot() compatible
function matplot(a1, a2, a3)
    [r,c]=size(a1);
    if nargin == 2
        % [t y1 y2 ... yN]
        t = a1(:,1);
        y = a1(:,2:c);
        cols = 1:(c-1);
        gain = 1;
        opt = a2;
    elseif nargin == 3
        if isvector(a1) & ismatrix(a2)
            % t, [y1 y2 .. yN]
            t = a1(:);
            cols = 1:numcols(a2);
            y = a2;
        elseif isempty(a1) & ismatrix(a2)
            % [], [y1 y2 .. yN]
            cols = 1:numcols(a2);
            y = a2;
            t = [1:numrows(y)]';
        end
        opt = a3;

    end
    t = t(:);
    [r,c]=size(y);
    sp = c*100 + 10;
    tmax = max(t);
    for i=1:c
        if (sp+i) == 111,
            clf
            plot(t,y(:,i));
            h(i) = gca;
        else
            h(i) = subplot(sp+i);
            plot(t,y(:,i));
        end
        set(h(i), 'UserData', i);
        set(h(i), 'Tag', 'mplot');
        v = axis;
        v(2) = tmax;
        axis(v);
        grid
        xlabel('Time');
            lab = sprintf('Y(%2d)', cols(i));

        ylabel(lab);
    end
    axes(h(1));
    figure(gcf)
end

function mlabel(lab, varargin)
    
    % find all child axes (subplots)
    h = findobj(gcf, 'Type', 'axes');
    
    for i=1:length(h),
        
        if strcmp( get(h(i), 'visible'), 'on'),
            axes(h(i))
            % get subplot number from user data (I don't know who
            % sets this but its very useful)
            sp = get(h(i), 'UserData');
            if sp == 1,
                topplot = sp;
            end
            ylabel(lab{sp}, varargin{:});
        end
    end
    
    if 0
        if nargin > 1,
            axes(h(topplot));	% top plot
            title(tit);
        end
    end
end

function mtools
    
    h = uicontextmenu;
    uimenu(h, 'Label', 'X zoom', 'CallBack', 'xaxis');
    uimenu(h, 'Label', '-->', 'CallBack', 'xscroll(0.5)');
    uimenu(h, 'Label', '<--', 'CallBack', 'xscroll(-0.5)');
    uimenu(h, 'Label', 'CrossHairs', 'CallBack', 'crosshair');
    uimenu(h, 'Label', 'X UNzoom', 'CallBack', 'unzoom');
    uimenu(h, 'Label', 'Pick delta', 'CallBack', 'fprintf(''%f %f\n'', diff(ginput(2)))');
    uimenu(h, 'Label', 'Line fit', 'CallBack', 'ilinefit');
    uimenu(h, 'Label', 'Show points', 'CallBack', 'showpoints(gca)');
    uimenu(h, 'Label', 'Apply X zoom to all', 'CallBack', 'xaxisall');
    for c=get(gcf, 'Children')',
        set(c, 'UIContextMenu', h);
        l = get(c, 'Children');
    end
    
    
    axes('pos', [0 0 1 0.05], 'visible', 'off')
    
end

function datestamp
  uicontrol('Style', 'text', ...
	 'String', date, ...
	'Units', 'Normalized', ...
	'HorizontalAlignment', 'Right', ...
	'BackgroundColor', 'w', ...
	'Position', [.8 0.97 .2 .03]);
end
