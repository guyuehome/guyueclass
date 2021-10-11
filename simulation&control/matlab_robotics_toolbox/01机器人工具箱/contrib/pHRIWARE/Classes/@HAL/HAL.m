%HAL Human Arm-like subclass for SerialLink
%
% A class to model the kinematics of the human right arm and human 
% arm-like robots. Is a subclass of the SerialLink class, developed by 
% Peter Corke (www.petercorke.com). See help folder for figure of HAL
% chain frames.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) hal = HAL(Tg, gre, erw, g)
%  (2) hal = HAL()
%
%  (2) is as per (1) but uses default values:
%       Tg = trotx(pi/2);
%       gre = from anthroData
%       erw = from anthroData
%       g = [-0.2; 0.1; 0.2];
%
% Outputs:
%  hal : Returned HAL object
%
% Inputs:
%  Tg  : Coordinate frame of the shoulder. If empty, reverts to default
%  gre : Length of the upper arm. If empty, reverts to default
%  erw : Length of the forearm. If empty, reverts to default
%  g   : Goal vector. If empty, reverts to default
%
% See also anthroData SerialLink

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
%
% RTB LIBRARY:
%
% Copyright (C) 1993-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license,
% Modified 15/6/2014 (HAL is a subclass of SerialLink)

classdef HAL < SerialLink

    properties (Dependent)
        AP % Arm properties (Tg, gre, erw, g)
    end
    
    properties
        goal % Elbow axis normal vector, for resolving swivel angle
		HAT % Matching CollisionModel object of human head and torso
		ARM % Matching CollisionModel object of human (right) arm
    end
    
    methods
        function hal = HAL(Tg, gre, erw, g)
   
            if ~nargin, [Tg, gre, erw, g] = deal([]); end
            if isempty(Tg), Tg = trotx(pi/2); end
            if isempty(gre), [~, gre] = anthroData('gre');end
            if isempty(erw), [~, erw] = anthroData('erw');end
            if isempty(g), g = [-0.2; 0.1; 0.2]; end
            
            wrh = 0; % Hand length, make non-zero in future version
            
            % Adduction/abduction
            L(1) = Revolute('alpha', -pi/2, 'offset', -pi/2,...
                'qlim', [-30 180]*d2r);
            % Extension/flexion
            L(2) = Revolute('alpha', -pi/2, 'offset', -pi/2,...
                'qlim', [-60 180]*d2r);
            % External/internal rotation
            L(3) = Revolute('d', gre, 'alpha', -pi/2, 'offset', pi,...
                'qlim', [-90 60]*d2r);
            % Extension/flexion
            L(4) = Revolute('a', erw,...
                'qlim', [-90 60]*d2r);
            % Ulnar/radial deviation
            L(5) = Revolute('alpha', pi/2,...
                'qlim', [-30 20]*d2r);
            % Extension/flexion
            L(6) = Revolute('alpha', -pi/2, 'offset', pi/2,...
                'qlim', [-80 60]*d2r);
            % Pronation/supination
            L(7) = Revolute('d', -wrh,...
                'qlim', [-80 80]*d2r);
            
            hal = hal@SerialLink(L, 'base', Tg,...
                'tool', troty(pi/2),...
                'name', 'Right Arm',...
                'comment', 'Kinematic model of the human right arm',...
                'ikine', 'RightArm');
            
            hal.manuf = '(C) Bryan Moutrie 2014';
            
            hal.goal = g;
            
            n = 50;
            Upoints = [zeros(1,n); linspace(0,2*gre/3,n); zeros(1,n)]';
            % Only 2/3 towards shoulder to not get caught
            Fpoints = [linspace(0,-erw,n); zeros(2,n)]';
            hal.points = {[], [], Upoints, Fpoints, [], [], []};
            hal.faces = cell(1,7);
        end
        
        function AP = get.AP(hal)
            AP = {hal.base, hal.d(3), hal.a(4), hal.goal};
        end
        
		function addCM(hal)
           %ADDCM Add CollisionModel objects for human head and torso
           % and arm.
           %
           % Syntax:
           %  (1) hal.addCM
           %
           % See also CollisionModel cmdl_arm cmdl_hat
           
            hal.HAT = cmdl_hat(hal.base);
			hal.ARM = cmdl_arm(hal.d(3), hal.a(4));
        end
        
        function varargout = plot3d(hal, q)
           %PLOT3D Plot HAL using CollisionModel data form HAT and ARM
           % properties. Overshadows SerialLink method of same name.
           %
           % Syntax:
           %  (1) hal.plot3d(q)
           %  (2) h = hal.plot3d(q)
           %
           %  (2) is as per (1) but returns graphics handles
           %
           % Outputs:
           %  h : 1x3 graphics handles - hat, upper arm, forearm
           %
           % Inputs:
           %  q : mx7 set of joint angles
           %
           % See also addCM SerialLink.plot3d
           
            if isempty(hal.HAT) || isempty(hal.ARM)
                error(pHRIWARE('error', 'Please add CollisionModels'));
            else
                hhat = hal.HAT.plot;
                Tu = hal.base * hal.A(1:3, q(1,:));
                Tf = Tu * hal.A(4, q(1,:));
                hu = hal.ARM.primitives{1}.plot('animate', Tu);
                hf = hal.ARM.primitives{2}.plot('animate', Tf);
                pause(0.1);
                % Animation for multiple poses:
                for p = 2: size(q,1)
                    Tu = hal.base * hal.A(1:3, q(p,:));
                    Tf = Tu * hal.A(4, q(p,:));
                    hal.ARM.primitives{1}.animate(hu, Tu);
                    hal.ARM.primitives{2}.animate(hf, Tf);
                    pause(0.1);
                end
                if nargout, varargout{1} = [hhat, hu, hf]; end
            end
        end

        function [q1, q2] = gikine(hal, Tu)
           %GIKINE Shoulder inverse kinematics of HAL right shoulder
           %
           % Wrapper function for HAL objects
           % (HAL.base replaces Tg)
           %
           % See also gikine
           
           [q1, q2] = gikine(hal.base, Tu);
        end
        
        function [Tf, Ts, Tu] = h2fsu(hal, varargin)
           %H2FSU Convert hand data to forearm, swivel, and upper arm frames
           %
           % Wrapper function for HAL objects
           % (HAL.AP replaces AP)
           %
           % See also h2fsu
           
           [Tf, Ts, Tu] = h2fsu(hal.AP, varargin{:});
        end
        
        function [q1, q2] = wikine(hal, varargin) %#ok<INUSL>
           %WIKINE Wrist inverse kinematics of HAL-like right wrist
           %
           % Wrapper function for HAL objects
           % (HAL is not used)
           %
           % See also wikine
           
           warning('wikine does not use a HAL object!');
           [q1, q2] = wikine(varargin{:});
        end
    end
    
end

