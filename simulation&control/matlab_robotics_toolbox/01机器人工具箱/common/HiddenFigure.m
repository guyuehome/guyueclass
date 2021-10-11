
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
classdef HiddenFigure

    properties
         h      % the graphics handle   
    end

    methods
        function hf = HiddenFigure(varargin)
            hf.h = figure('HandleVisibility', 'off', varargin{:});
        end

        function clf(hf)
            clf(hf.h)
        end

        function axes(hf, varargin)
            axes('Parent', hf.h, varargin{:});
        end

        function axis(hf, varargin)
            axis(hf.h, varargin{:});
        end

        function plot(hf, varargin)
            hf.axes();
            plot(varargin{:});
        end

        function title(hf, name)
            set(hf.h, 'Name', name);
        end
    end
end
