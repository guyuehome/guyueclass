%COLORNAME Map between color names and RGB values
%
% RGB = COLORNAME(NAME) is the RGB-tristimulus value (1x3) corresponding to
% the color specified by the string NAME.  If RGB is a cell-array (1xN) of
% names then RGB is a matrix (Nx3) with each row being the corresponding
% tristimulus.
%
% XYZ = COLORNAME(NAME, 'xyz') as above but the XYZ-tristimulus value 
% corresponding to the color specified by the string NAME.
%
% XY = COLORNAME(NAME, 'xy') as above but the xy-chromaticity coordinates 
% corresponding to the color specified by the string NAME.
%
% NAME = COLORNAME(RGB) is a string giving the name of the color that is 
% closest (Euclidean) to the given RGB-tristimulus value (1x3).  If RGB is
% a matrix (Nx3) then return a cell-array (1xN) of color names.
%
% NAME = COLORNAME(XYZ, 'xyz') as above but the color is the closest (Euclidean)
% to the given XYZ-tristimulus value.
%
% NAME = COLORNAME(XYZ, 'xy') as above but the color is the closest (Euclidean)
% to the given xy-chromaticity value with assumed Y=1.
%
% Notes::
% - Color name may contain a wildcard, eg. "?burnt"
% - Based on the standard X11 color database rgb.txt.
% - Tristimulus values are in the range 0 to 1



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

function r = colorname(a, varargin)

    opt.color = {'rgb', 'xyz', 'xy'};
    opt = tb_optparse(opt, varargin);

    persistent  rgbtable;
    
    mvtb_present = exist('tristim2cc');
    
    % ensure that the database is loaded
    if isempty(rgbtable)
        % load mapping table from file
        fprintf('loading rgb.txt\n');
        f = fopen('private/rgb.txt', 'r');
        k = 0;
        rgb = [];
        names = {};
        xy = [];
        
        while ~feof(f),
            line = fgets(f);
            if line(1) == '#',
                continue;
            end
            
            [A,count,errm,next] = sscanf(line, '%d %d %d');
            if count == 3
                k = k + 1;
                rgb(k,:) = A' / 255.0;
                names{k} = lower( strtrim(line(next:end)) );
                if mvtb_present
                    xy = tristim2cc( colorspace('RGB->XYZ', rgb) );
                end
            end
        end
        s.rgb = rgb;
        s.names = names;
        if mvtb_present
            s.xy = xy;
        end
        rgbtable = s;
    end
    
    if isstr(a)
        % map name to rgb/xy
        if a(1)  == '?' 
            % just do a wildcard lookup
            r = namelookup(rgbtable, a(2:end), opt);
        else
            r = name2rgb(rgbtable, a, opt);
        end
    elseif iscell(a)
        % map multiple names to rgb
        r = [];
        for name=a,
            rgb = name2rgb(rgbtable, name{1}, opt.xy);
            if isempty(rgb)
                warning('Color %s not found', name{1});
            end
            r = [r; rgb];
        end
    else
        % map values to strings
        switch opt.color
            case 'rgb'
                if numel(a) == 3
                    r = rgb2name(rgbtable, a(:)');
                elseif numcols(a) ~= 3
                    error('RGB data must have 3 columns');
                else
                    r = {};
                    for i=1:numrows(a)
                        r{i} = rgb2name(rgbtable, a(i,:));
                    end
                end
                
            case 'xyz'
                if numel(a) == 3
                    rgb = colorspace('XYZ->RGB', a(:)');
                    r = rgb2name(rgbtable, rgb);
                elseif numcols(a) ~= 3
                    error('XYZ data must have 3 columns');
                else
                    rgb = colorspace('XYZ->RGB', a);
                    
                    r = {};
                    for i=1:numrows(a)
                        r{i} = rgb2name(rgbtable, rgb(i,:));
                    end
                end
                
            case 'xy'
                if numel(a) == 2
                    Y = 1;  XYZ = 1/a(2);
                    X = a(1) * XYZ;
                    Z = (1-a(1)-a(2)) * XYZ;
                    rgb = colorspace('XYZ->RGB', [X Y Z]);
                    r = rgb2name(rgbtable, rgb);
                elseif numcols(a) ~= 2
                    error('xy data must have 2 columns');
                else
                    Y = ones(numrows(a),1);  XYZ = 1./a(:,2);
                    X = a(:,1) .* XYZ;
                    Z = (1-a(:,1)-a(:,2)) .* XYZ;
                    rgb = colorspace('XYZ->RGB', [X Y Z]);
                    
                    r = {};
                    for i=1:numrows(a)
                        r{i} = rgb2name(rgbtable, rgb(i,:));
                    end
                end
        end
         

    end
end
    
function r = namelookup(table, s)
    s = lower(s);   % all matching done in lower case
    
    r = {};
    count = 1;
    for k=1:length(table.names),
        if ~isempty( findstr(table.names{k}, s) )
            r{count} = table.names{k};
            count = count + 1;
        end
    end
end

function out = name2rgb(table, s, opt)

    s = lower(s);   % all matching done in lower case
    
    for k=1:length(table.names),
        if strcmp(s, table.names(k)),
            rgb = table.rgb(k,:);
            switch opt.color
                case 'rgb'
                    out = rgb;
                case 'xy'
                    XYZ = colorspace('RGB->XYZ', r);
                    out = tristim2cc(XYZ);
                case 'xyz'
                    out = colorspace('RGB->XYZ', rgb);
            end
            return;
        end
    end
    out = [];
end

function r = rgb2name(table, v)
    d = table.rgb - ones(numrows(table.rgb),1) * v;
    n = colnorm(d');
    [z,k] = min(n);
    r = table.names{k};
end

function r = xy2name(table, v)
    d = table.xy - ones(numrows(table.xy),1) * v;
    n = colnorm(d');
    [z,k] = min(n);
    r = table.names{k};
end
