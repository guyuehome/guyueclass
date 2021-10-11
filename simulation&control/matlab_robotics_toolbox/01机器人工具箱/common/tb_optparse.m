%OPTPARSE Standard option parser for Toolbox functions
%
% OPTOUT = TB_OPTPARSE(OPT, ARGLIST) is a generalized option parser for
% Toolbox functions.  OPT is a structure that contains the names and
% default values for the options, and ARGLIST is a cell array containing
% option parameters, typically it comes from VARARGIN.  It supports options
% that have an assigned value, boolean or enumeration types (string or
% int).
%
% The software pattern is:
%
%       function(a, b, c, varargin)
%       opt.foo = false;
%       opt.bar = true;
%       opt.blah = [];
%       opt.choose = {'this', 'that', 'other'};
%       opt.select = {'#no', '#yes'};
%       opt = tb_optparse(opt, varargin);
%
% Optional arguments to the function behave as follows:
%   'foo'           sets opt.foo := true
%   'nobar'         sets opt.foo := false
%   'blah', 3       sets opt.blah := 3
%   'blah',{x,y}    sets opt.blah := {x,y}
%   'that'          sets opt.choose := 'that'
%   'yes'           sets opt.select := (the second element)
%
% and can be given in any combination.
%
% If neither of 'this', 'that' or 'other' are specified then opt.choose := 'this'.
% Alternatively if:
%        opt.choose = {[], 'this', 'that', 'other'};
% then if neither of 'this', 'that' or 'other' are specified then opt.choose := []
%
% If neither of 'no' or 'yes' are specified then opt.select := 1.
%
% Note:
% - That the enumerator names must be distinct from the field names.
% - That only one value can be assigned to a field, if multiple values
%   are required they must placed in a cell array.
% - To match an option that starts with a digit, prefix it with 'd_', so
%   the field 'd_3d' matches the option '3d'.
% - OPT can be an object, rather than a structure, in which case the passed
%   options are assigned to properties.
%
% The return structure is automatically populated with fields: verbose and
% debug.  The following options are automatically parsed:
%   'verbose'       sets opt.verbose := true
%   'verbose=2'     sets opt.verbose := 2 (very verbose)
%   'verbose=3'     sets opt.verbose := 3 (extremeley verbose)
%   'verbose=4'     sets opt.verbose := 4 (ridiculously verbose)
%   'debug', N      sets opt.debug := N
%   'showopt'       displays opt and arglist
%   'setopt',S      sets opt := S, if S.foo=4, and opt.foo is present, then
%                   opt.foo is set to 4.
%
% The allowable options are specified by the names of the fields in the
% structure opt.  By default if an option is given that is not a field of 
% opt an error is declared.  
%
% [OPTOUT,ARGS] = TB_OPTPARSE(OPT, ARGLIST) as above but returns all the
% unassigned options, those that don't match anything in OPT, as a cell
% array of all unassigned arguments in the order given in ARGLIST.
%
% [OPTOUT,ARGS,LS] = TB_OPTPARSE(OPT, ARGLIST) as above but if any
% unmatched option looks like a MATLAB LineSpec (eg. 'r:') it is placed in LS rather
% than in ARGS.
%


% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
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

% Modifications by Joern Malzahn to support classes in addition to structs

function [opt,others,ls] = tb_optparse(in, argv)

    if nargin == 1
        argv = {};
    end

    if ~iscell(argv)
        error('RTB:tboptparse:badargs', 'input must be a cell array');
    end

    arglist = {};

    argc = 1;
    opt = in;
    
    if ~isfield(opt, 'verbose')
        opt.verbose = false;
    end
    if ~isfield(opt, 'debug')
        opt.debug = 0;
    end

    showopt = false;
    choices = [];

    while argc <= length(argv)
        % index over every passed option
        option = argv{argc};
        assigned = false;
        
        if isstr(option)

            switch option
            % look for hardwired options
            case 'verbose'
                opt.verbose = true;
                assigned = true;
            case 'verbose=2'
                opt.verbose = 2;
                assigned = true;
            case 'verbose=3'
                opt.verbose = 3;
                assigned = true;
            case 'verbose=4'
                opt.verbose = 4;
                assigned = true;
            case 'debug'
                opt.debug = argv{argc+1};
                argc = argc+1;
                assigned = true;
            case 'setopt'
                new = argv{argc+1};
                argc = argc+1;
                assigned = true;

                % copy matching field names from new opt struct to current one
                for f=fieldnames(new)'
                    if isfield(opt, f{1})
                        opt.(f{1}) = new.(f{1});
                    end
                end
            case 'showopt'
                showopt = true;
                assigned = true;

            otherwise
                % does the option match a field in the opt structure?
%                 if isfield(opt, option) || isfield(opt, ['d_' option])
%                if any(strcmp(fieldnames(opt),option)) || any(strcmp(fieldnames(opt),))
                 if isfield(opt, option) || isfield(opt, ['d_' option]) || isprop(opt, option)
                    
                    % handle special case if we we have opt.d_3d, this
                    % means we are looking for an option '3d'
                    if isfield(opt, ['d_' option]) || isprop(opt, ['d_' option])
                        option = ['d_' option];
                    end
                    
                    %** BOOLEAN OPTION
                    val = opt.(option);
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt.(option) = true;
                    else
                        %** OPTION IS ASSIGNED VALUE FROM NEXT ARG
                        % otherwise grab its value from the next arg
                        try
                            opt.(option) = argv{argc+1};
                        catch me
                            if strcmp(me.identifier, 'MATLAB:badsubscript')
                                error('RTB:tboptparse:badargs', 'too few arguments provided for option: [%s]', option);
                            else
                                rethrow(me);
                            end
                        end
                        argc = argc+1;
                    end
                    assigned = true;
                elseif length(option)>2 && strcmp(option(1:2), 'no') && isfield(opt, option(3:end))
                    %* BOOLEAN OPTION PREFIXED BY 'no'
                    val = opt.(option(3:end));
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt.(option(3:end)) = false;
                        assigned = true;
                    end
                else
                    % the option doesn't match a field name
                    % let's assume it's a choice type
                    %     opt.choose = {'this', 'that', 'other'};
                    %
                    % we need to loop over all the passed options and look
                    % for those with a cell array value
                    for field=fieldnames(opt)'
                        val = opt.(field{1});
                        if iscell(val)
                            for i=1:length(val)
                                if isempty(val{i})
                                    continue;
                                end
                                % if we find a match, put the final value
                                % in the temporary structure choices
                                %
                                % eg. choices.choose = 'that'
                                %
                                % so that we can process input of the form
                                %
                                %  'this', 'that', 'other'
                                %
                                % which should result in the value 'other'
                                if strcmp(option, val{i})
                                    choices.(field{1}) = option;
                                    assigned = true;
                                    break;
                                elseif val{i}(1) == '#' && strcmp(option, val{i}(2:end))
                                    choices.(field{1}) = i;
                                    assigned = true;
                                    break;
                                end
                            end
                            if assigned
                                break;
                            end
                        end
                    end
                end
            end % switch
        end
        if ~assigned
            % non matching options are collected
            if nargout >= 2
                arglist = [arglist argv(argc)];
            else
                if isstr(argv{argc})
                    error(['unknown options: ' argv{argc}]);
                end
            end
        end
        
        argc = argc + 1;
    end % while
    
    % copy choices into the opt structure
    if ~isempty(choices)
        for field=fieldnames(choices)'
           opt.(field{1}) = choices.(field{1});
        end
    end

    % if enumerator value not assigned, set the default value
    if ~isempty(in)
        for field=fieldnames(in)'
            if iscell(in.(field{1})) && iscell(opt.(field{1}))
                val = opt.(field{1});
                if isempty(val{1})
                    opt.(field{1}) = val{1};
                elseif val{1}(1) == '#'
                    opt.(field{1}) = 1;
                else
                    opt.(field{1}) = val{1};
                end
            end
        end
    end
                        
    if showopt
        fprintf('Options:\n');
        opt
        arglist
    end

    if nargout == 3
        % check to see if there is a valid linespec floating about in the
        % unused arguments
        for i=1:length(arglist)
            s = arglist{i};
            % get color
            [b,e] = regexp(s, '[rgbcmywk]');
            s2 = s(b:e);
            s(b:e) = [];
            
            % get line style
            [b,e] = regexp(s, '(--)|(-.)|-|:');
            s2 = [s2 s(b:e)];
            s(b:e) = [];
            
            % get marker style
            [b,e] = regexp(s, '[o\+\*\.xsd\^v><ph]');
            s2 = [s2 s(b:e)];
            s(b:e) = [];
            
            % found one
            if isempty(s)
                ls = arglist{i};
                arglist(i) = [];
                others = arglist;
                break;
            end
        end
        ls = [];
        others = arglist;
    elseif nargout == 2
        others = arglist;
    end
