%Plucker Plucker coordinate class
%
% Concrete class to represent a line in Plucker coordinates.
%
% Methods::
% line    Return Plucker line coordinates (1x6)
% side    Side operator
%
% Operators::
% *       Multiple Plucker matrix by a general matrix
% |       Side operator
%
%
% Notes::
% - This is reference class object
% - Link objects can be used in vectors and arrays

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

classdef Plucker < handle

    properties
        L   % 4x4 Plucker matrix
    end

    methods
        function pl = Plucker(p1, p2)
            %Plucker.Plucker Create Plucker object
            %
            % P = Plucker(P1, P2) create a Plucker object that represents
            % the line joining the 3D points P1 (3x1) and P2 (3x1).

            % convert 3D points to homogeneous form
            p1 = [p1(:); 1]; p2 = [p2(:); 1];

            % create the Plucker matrix
            pl.L = p1*p2' - p2*p1';
        end

        function v = line(pl)
            %Plucker.double Plucker liner coordinates
            %
            % P.line() is a 6-vector representation of the Plucker
            % coordinates of the line.

            L = pl.L;
            v = [L(2,1) L(3,1) L(4,1) L(4,3) L(2,4) L(3,2)];
        end

        function z = mtimes(a1, a2)
            %Plucker.mtimes Plucker composition
            %
            % P * M is the product of the Plucker matrix and M (4xN).
            %
            % M * P is the product of M (Nx4) and the Plucker matrix.

            if isa(a1, 'Plucker')
                if numrows(a2) ~= 4
                    error('RTB:Plucker:badarg', 'must postmultiply by 4xN matrix');
                end
                z = a1.L * a2;
            elseif isa(a2, 'Plucker')
                if numcols(a1) ~= 4
                    error('RTB:Plucker:badarg', 'must premultiply by Nx4 matrix');
                end
                z = a1 * a2.L;
            end
        end

        function z = or(pl1, pl2)
            % P1 | P2 is the side operator which is zero whenever
            % the lines P1 and P2 intersect or are parallel.
            z = side(pl1, pl2);
        end

        function z = side(pl1, pl2)
            %Plucker.mtimes Side operator
            %
            % SIDE(P1, P2) is the side operator which is zero whenever
            % the lines P1 and P2 intersect or are parallel.
            if ~isa(pl2, 'Plucker')
                error('RTB:Plucker:badarg', 'both arguments to | must be Plucker objects');
            end
            L1 = pl1.line(); L2 = pl2.line();

            z = L1([1 5 2 6 3 4]) * L2([5 1 6 2 4 3])';
        end

        function display(pl)
            %Plucker.display Display parameters
            %
            % P.display() displays the Plucker parameters in compact single line format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Plucker object and the command has no trailing
            %   semicolon.
            %
            % See also Plucker.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(pl) );
        end % display()

        function s = char(pl)
            %Plucker.char Convert to string
            %
            % s = P.char() is a string showing Plucker parameters in a compact single 
            % line format.
            %
            % See also Plucker.display.
            s = '';
            for i=1:length(pl)
                
                ps = ['{', num2str(pl.line()), '}'];
                if isempty(s)
                    s = ps;
                else
                    s = char(s, ps);
                end
            end
        end

    end % methods
end % class
