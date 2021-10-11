%WIKINE Wrist inverse kinematics of HAL-like right wrist
%
% Computes the inverse kinematics of the right wrist which is the 
% same kinematically as a HAL object. Either the hand and wrist frames
% can be entered separately, or a relative rotation used.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Syntax:
%  (1) [q1, q2] = wikine(Tf, Th)
%  (2) [q1, q2] = wikine(fTh)
%
%  (2) is as per (1) but the transform of the hand frame, relative to
%       the forearm frame, is given explicitly
%
% Outputs:
%  q1 : First family of solutions (mx3 matrix where m = size(Th,3))
%  q2 : Second family of solutions (mx3 matrix where m = size(Th,3))
%
% Inputs:
%  Tf  : Transformation matrix of the forearm frame. May be a 4x4xm
%         series of frames (or higher order, which is compressed to 3D)
%  Th  : Transformation matrix of the hand frame. May be a 4x4xm series
%         of frames (or higher order, which is compressed to 3D)
%  fTh : Transformation matrix of the wrist (Tf \ Th). May be a 4x4xm
%         series of frames (or higher order, which is compressed to 3D)
%
% See also HAL gikine HAL.ikine

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

function [q1, q2] = wikine(varargin)

if length(varargin) == 2
    Tf = varargin{1};
    Th = varargin{2};
    
    Tw_sz = size(Th);
    
    if length(Tw_sz) > 2
        x = prod(Tw_sz(3:end));
    else
        x = 1;
    end
    
    fRh = zeros(3,3,x);
    for i = 1:x
        fRh(:,:,i) = Tf(1:3,1:3,i)' * Th(1:3,1:3,i);
    end 
else
    fRh = varargin{1}(1:3,1:3,:);
end

%Z,-Y,X Tait-Bryan angles

% Family 1:
q1(:,1) = squeeze(atan2(fRh(2,1,:), fRh(1,1,:)));
q1(:,2) = squeeze(asin(fRh(3,1,:)));
q1(:,3) = squeeze(atan2(-fRh(3,2,:), fRh(3,3,:)));

% Make -pi values pi so that they are deemed within joint limits
fix = q1 <= (-pi+1e-4);
q1(fix) = pi;

% Family 2:
q2(:,1) = squeeze(atan2(-fRh(2,1,:), -fRh(1,1,:)));
q2(:,2) = pi-q1(:,2);
q2(:,3) = squeeze(atan2(-fRh(3,2,:), -fRh(3,3,:)));
end

