%RUNSCRIPT Run an M-file in interactive fashion
%
% RUNSCRIPT(FNAME, OPTIONS) runs the M-file FNAME and pauses after every
% executable line in the file until a key is pressed.  Comment lines are shown
% without any delay between lines.
%
% Options::
% 'delay',D    Don't wait for keypress, just delay of D seconds (default 0)
% 'cdelay',D   Pause of D seconds after each comment line (default 0)
% 'begin'      Start executing the file after the comment line %%begin (default false)
% 'dock'       Cause the figures to be docked when created
% 'path',P     Look for the file FNAME in the folder P (default .)
% 'dock'       Dock figures within GUI
%
% Notes::
% - If no file extension is given in FNAME, .m is assumed.
% - If the executable statement has comments immediately afterward (no blank lines)
%   then the pause occurs after those comments are displayed.
% - A simple '-' prompt indicates when the script is paused, hit enter.
% - If the function cprintf() is in your path, the display is more
%   colorful, you can get this file from MATLAB Central.
% - If the file has a lot of boilerplate, you can skip over and not display
%   it by giving the 'begin' option which searchers for the first line
%   starting with %%begin and commences execution at the line after that.
%
% See also eval.


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

function runscript(fname, varargin)
    
    opt.path = [];
    opt.delay = [];
    opt.begin = false;
    opt.cdelay = 0;
    opt.dock = false;
    
    opt = tb_optparse(opt, varargin);
        
    close all
    
    prevDockStatus = get(0,'DefaultFigureWindowStyle');
    if opt.dock
        set(0,'DefaultFigureWindowStyle','docked');
    else
        set(0,'DefaultFigureWindowStyle','normal');
    end

    
    if ~isempty(opt.path)
        fname = fullfile(opt.path, [fname '.m']);
    else
        fname = [fname '.m'];
    end
    
    fp = fopen(fname, 'r');
    
    clc
    fprintf('--- runscript <-- %s\n', fname);
    
    running = false;
    shouldPause = false;
    savedText = [];
    
    if ~opt.begin
        running = true;
    end
    
    lineNum = 1;
    
    % stashMode
    %  0 normal
    %  1 loop
    %  2 continuation
    continMode = false;
    compoundDepth = 0;
    
    while 1
        % get the next line from the file, bail if EOF
        line = fgetl(fp);
        if line == -1
            break
        end
        lineNum = lineNum+1;
        
        % logic to skip lines until we see one beginning with %%begin
        if ~running
            if strcmp(line, '%%begin')
                running = true;
            else
                continue;
            end
        end;
        
        if length(strtrim(line)) == 0
            % blank line
            fprintf('\n');
            if shouldPause
                scriptwait(opt);
                shouldPause = false;
            end
            continue
        elseif startswith(strtrim(line), '%')
            % line was a comment
            disp(line)
            pause(opt.cdelay)  % optional comment delay
            continue;
        else
            if shouldPause
                scriptwait(opt);
                shouldPause = false;
            end
        end
        
        % if the start of a loop, stash the text for now
        if startswith(line, 'for') || startswith(line, 'while') || startswith(line, 'if')
            % found a compound block, don't eval it until we get to the end
            compoundDepth = compoundDepth + 1;
        end
        % if the statement has a continuation
        if endswith(line, '...')  && compoundDepth == 0
            % found a compound statement, don't eval it until we get to the end
            continMode = true;
        end
        
        if compoundDepth == 0 && ~continMode
            prompt = '>> ';
        else
            prompt = '';
        end
        
        % display the line with a pretend MATLAB prompt
        if exist('cprintf')
            cprintf('blue', '%s%s\n', prompt, line)
        else
            fprintf('%s', prompt); disp(line)
        end
        
        if compoundDepth > 0 || continMode
            % we're in stashing mode
            savedText = strcat(savedText, '\n', line);
        end
        
        if compoundDepth > 0 && startswith(line, 'end')
            % the compound block is fully unnested
            
            compoundDepth = compoundDepth - 1;
            if compoundDepth == 0
                evalSavedText(savedText, lineNum);
                savedText = '';
                shouldPause = true;
            end
            continue
            
        elseif continMode && ~endswith(line, '...')
            % no longer in continuation mode
            
            evalSavedText(savedText, lineNum);
            savedText = '';
            continMode = false;
            shouldPause = true;
            continue
        end
        
        if compoundDepth == 0 && ~continMode
            % it's a simple executable statement, execute it
            fprintf(' \n');
            evalSavedText(line, lineNum);
            shouldPause = true;
        end
    end
    fprintf('------ done --------\n');
    % restore the docking mode if we set it
    set(0,'DefaultFigureWindowStyle', prevDockStatus)
end

    function evalSavedText(text, lineNum)
        if length(strtrim(text)) == 0
            return
        end
        
        text = sprintf(text);
        
        try
            evalin('base', text);
        catch m
            fprintf('error in script %s at line %d', lineNum);
            m.rethrow();
        end
    end

% delay or prompt according to passed options
function scriptwait(opt)
    if isempty(opt.delay)
        %a = input('-', 's');
        prompt = 'continue?';
        if exist('cprintf')
            cprintf('red', prompt); pause;
            for i=1:length(prompt)
                cprintf('text', '\b');
            end
        else
            fprintf(prompt); pause;
            for i=1:length(prompt)
                fprintf('\b');
            end
        end
    else
        pause(opt.delay);
    end
end

% test if s2 is at the start of s1 (ignoring leading spaces)
function res = startswith(s1, s2)

    s1 = strtrim(s1);  % trim leading white space
    r = strfind(s1, s2);
    res = false;
    if ~isempty(r) && (r(1) == 1)
        res = true;
    end
end

% test if s2 is at the end of s1
function res = endswith(s1, s2)

    if length(s1) < length(s2)
        res = false;
    else
        n2 = length(s2)-1;
        res = strcmp(s1(end-n2:end), s2);
    end

end
    
