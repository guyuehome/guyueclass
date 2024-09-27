%pHRIWARE physical HRI worspace analysis, research and evaluation
%
% pHRIWARE (pron. 'freeware') provides tools to analyse, research and 
% evaluate physical human-robot interactions. Many of these tools also
% requires the Robotics Toolbox for MATLAB(R) (RTB), developed by Peter
% Corke. This may be downloaded at www.petercorke.com.
%
% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.

function varargout = pHRIWARE(varargin)

switch length(varargin)
    case 0
        % General information
        help pHRIWARE.m
        fprintf('  '); pHRIWARE('C');
        fprintf(['  This version: ',pHRIWARE('ver'),'\n\n']);
        
        str = input(' See some demos? [y/n] >> ', 's');
        switch str
            case 'y'
                demos_pHRIWARE;
            case 'n'
                disp('Enjoy using pHRIWARE!');
            otherwise
                error(pHRIWARE('error','inputValue'));
        end
        
    case 1
        % Copyright notice
        if strcmp(varargin{1},'c') || strcmp(varargin{1},'C')
            d8 = date;
            year = d8(end-3:end);
            copy = ['pHRIWARE is Copyrighted by Bryan Moutrie ', ...
                '(2013-',year,') (',varargin{1},')'];
            if ~nargout, disp(copy); else varargout{1} = copy; end
            
        % Version/subversion    
        elseif strcmp(varargin{1},'ver')
            curdir = cd;
            loc = fileparts(which('pHRIWARE.m'));
            cd(loc);
            verfile = fopen('VER.txt');
            full = fscanf(verfile,'%s');
            fclose(verfile);
            cd(curdir);
            if nargout == 2,
                dot = find(full == '.');
                varargout = {str2num(full(1:dot-1)), ...
                    str2num(full(dot+1:end))}; %#ok<*ST2NM>
            else
                varargout{1} = full;
            end
            
        % Defined colours    
        elseif strcmp(varargin{1},'blue')
            varargout{1} = [91 155 213]/255;
        elseif strcmp(varargin{1},'navy')
            varargout{1} = pHRIWARE('blue')/2;
        elseif strcmp(varargin{1},'orange')
            varargout{1} = [237 125 49]/255;
        elseif strcmp(varargin{1},'brown')
            varargout{1} = pHRIWARE('orange')/2;
        elseif strcmp(varargin{1},'skin')
            varargout{1} = [1 0.9 0.8];
        
        % Invalid
        else
            error(pHRIWARE('error', 'inputValue'));
        end
        
    case 2
        % Error and warning messages
        if strcmp(varargin{1},'error')
            type = varargin{2};
            switch type
                case 'inputSize'
                    message = 'An input has an invalid size';
                case 'inputType'
                    message = 'An input has an invalid type';
                case 'inputValue'
                    message = 'An input has an invalid value';
                case 'MuPAD'
                    message = 'MuPAD is not installed or not workings';
                case 'numInputs'
                    message = 'The number of inputs is invalid';
                otherwise
                    message = type;
                    type = 'custom';
            end

            varargout{1} = struct('indentifier', ['pHRIWARE:',type],...
                'message', message);
            
        % Invalid    
        else
            error(pHRIWARE('error', 'inputValue'));
        end
        
    otherwise
        % Invalid
        error(pHRIWARE('error', 'numInputs'));
end
end

