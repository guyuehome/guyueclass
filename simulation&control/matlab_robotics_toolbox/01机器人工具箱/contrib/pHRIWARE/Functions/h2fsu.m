%H2FSU Convert hand data to forearm, swivel, and upper arm frames
% 
% Returns the corresponding forearm, swivel and upper arm frame(s) for
% given hand frame(s) or point(s). If hand data is be given in a N-D
% array, the outputs will be of same shape. The swivel angle can be
% resolved using a number of methods (see Inputs). See help folder for
% figure of relevant frames.
% 
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
% 
% This file modifies file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
% 
% Syntax:
%  (1) [Tf, Ts, Tu] = h2fsu(AP, Th, phi)
%  (2) [Tf, Ts, Tu] = h2fsu(AP, Th)
%  (3) [Tf, Ts, Tu] = h2fsu(AP, ph, phi)
%  (4) [Tf, Ts, Tu] = h2fsu(AP, ph)
%  (5) [Tf, Ts, Tu] = h2fsu(AP, x, y, z, phi)
%  (6) [Tf, Ts, Tu] = h2fsu(AP, x, y, z)
% 
%  (2) is as per (1) with phi := (1)
%  (4) is as per (3) with phi := (2)
%  (5) is as per (3) with p := 3xlength(x)xlength(y)xlength(z), so that
%       ph is every permutation of the elements in vectors x, y, and z
%  (6) is as per (5) with phi := (2)
% 
% Outputs:
%  Tf : Forearm frames (4x4x...)
%  Ts : Swivel frames (4x4x...)
%  Tu : Upper arm frames (4x4x...)
% 
% Inputs:
%  AP  : Cell containing arm properties, {Tg, gre, erw, g}
%  Th  : Hand frame(s) (4x4x...)
%  ph  : Hand point(s) (3x...)
%  x   : Hand x coordinates (vector)
%  y   : Hand y coordinates (vector)
%  z   : Hand z coordinates (vector)
%  phi : Swivel angle,
%        (0) 0     : Uses phi = 0
%        (1)       : Uses the z-axis of Tw for the z-axis of Ts, via
%                     projection
%        (2) 'h2g' : Is the hand-2-goal method, s.t. z-axis of Ts is 
%                     the cross product of the swivel axis and g
%        (3) step  : step is a scalar >0 so that the swivel angle
%                      is added as an an extra dimension, with phi = 
%                      median human range +/- step increments up to the
%                      human limits. The swivel angle increases with 
%                      the increasing index of the extra dimension.
%        (4) PHI   : PHI uses an explicit value, the number of elements
%                     in PHI must be a multiple of the number of hand 
%                     points/frames
% 
% See also HAL HAL.h2fsu HAL.ikine

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
% Modified 15/6/2014 (input checking removed, formatting modified)

function [Tf, Ts, Tu] = h2fsu(AP, varargin)

Tg = AP{1};
gre = AP{2};
erw = AP{3};
pg = Tg(1:3,4);
g = [0; 0; 1]; % Default value for phi = 0, is overwritten if neccesary

% Get hand data, assign default phi type
switch length(varargin)
    case {1 2} % Points or frames
        handData = varargin{1};
        data_sz = size(handData);
        if isequal(data_sz(1:2), [4 4])
            ph = squeeze(handData(1:3,4,:));
            data_sz = data_sz(3:end);
            if isempty(data_sz), data_sz = 1; end
            elbowAxisd = squeeze(handData(1:3,3,:));
            phiType = 1; % Projection
        elseif data_sz(1) == 3
            ph = reshape(handData,3,[]);
            data_sz = data_sz(2:end);
            g = AP{4};
            phiType = 2; % Hand-2-goal
        else
            error(pHRIWARE('error', 'inputSize'));
        end
        phi_sz = 1;      
        
    case {3 4} % XYZ
        x = varargin{1};    y = varargin{2};    z = varargin{3};
        [X, Y, Z] = ndgrid(x,y,z);
        data_sz = [length(x), length(y), length(z)];
        ph = [X(:), Y(:), Z(:)]';
        phiType = 2; % Hand-2-goal
        phi_sz = 1;
end

gpw = [ph(1,:)-pg(1); ph(2,:)-pg(2); ph(3,:)-pg(3)];
grw = sqrt(sum(gpw.^2, 1));
grw((gre+erw) < grw|grw <= abs(gre-erw)) = NaN; % Unreachable poses
ers = (1./(2*grw)).*sqrt( (4*grw.^2*gre^2) - (grw.^2-erw^2+gre^2).^2 );
grs = sqrt(gre^2-ers.^2);
srw = sqrt(erw^2-ers.^2); 

swivelAxis(3,:) = gpw(3,:) ./ grw;
swivelAxis(2,:) = gpw(2,:) ./ grw;
swivelAxis(1,:) = gpw(1,:) ./ grw;

% Parse phi, if given
switch length(varargin)
    case {2 4}
        phi = varargin{end};
        if isequal(phi,0)
            phiType = 0; % phi = Zero
            phi_sz = 1;
        elseif isequal(phi,'h2g')
            phiType = 2; % phi = Hand-2-goal
            g = AP{4};
            phi_sz = 1;
        elseif isscalar(phi)
            if phi < 0, error(pHRIWARE('error', 'inputValue')); end
            if isnan(phi), error(pHRIWARE('error', 'inputValue')); end
            phiType = 3; % Median +/- steps
            load('SwivelData.mat');
            ind = linspace(-1,1,size(phi_med,1));
            phim = interp3(ind,ind,ind,phi_med,gpw(2,:)/(gre+erw),...
                gpw(1,:)/(gre+erw),gpw(3,:)/(gre+erw));
            phir = interp3(ind,ind,ind,phi_range,gpw(2,:)/(gre+erw),...
                gpw(1,:)/(gre+erw),gpw(3,:)/(gre+erw));
            % Do not know why x and y need to be swapped.....
            phi_steps = floor(phir/phi);
            mps = max(phi_steps);
            phi_sz = mps*2+1;
            phi_num = NaN(length(phim),phi_sz);
            phi_num(:,mps+1) = phim;
            for step = 1:mps
                phi_num(phi_steps>=step,mps+1+step) = ...
                    phim(phi_steps>=step)+step*phi;
                phi_num(phi_steps>=step,mps+1-step) = ...
                    phim(phi_steps>=step)-step*phi;
            end
            phi = phi_num;
        elseif mod(numel(phi),prod(data_sz)) == 0
            phiType = 4; % Explicit
            phi = reshape(phi,prod(data_sz),[]);
            phi_sz = size(phi,2);
        else
            error(pHRIWARE('error', 'inputSize'));
        end
        data_sz = [data_sz, phi_sz];
end

% Create elbow axes
if phiType == 1 % Projection
        eDOTs = dot(elbowAxisd,swivelAxis);
        elbowAxis(1,:) = elbowAxisd(1,:) - swivelAxis(1,:) .* eDOTs;
        elbowAxis(2,:) = elbowAxisd(2,:) - swivelAxis(2,:) .* eDOTs;
        elbowAxis(3,:) = elbowAxisd(3,:) - swivelAxis(3,:) .* eDOTs;
else % Goal
        G = skew(g);
        elbowAxis = -G*swivelAxis;
end

elbowAxis_norm = sqrt(sum(elbowAxis.^2, 1));

% Find singular values:
sing = find(elbowAxis_norm < eps);
switch phiType
    case {1 2}
        elbowAxis_norm(sing) = NaN;
    otherwise
        elbowAxis_norm(sing) = 1;
        elbowAxis(1,sing) = Tg(1,1);
        elbowAxis(2,sing) = Tg(2,1);
        elbowAxis(3,sing) = Tg(3,1);
end

elbowAxis(1,:) = elbowAxis(1,:) ./ elbowAxis_norm;
elbowAxis(2,:) = elbowAxis(2,:) ./ elbowAxis_norm;
elbowAxis(3,:) = elbowAxis(3,:) ./ elbowAxis_norm;

ys = cross(elbowAxis,swivelAxis);
nan = any(isnan(ys),1); % Logical
notnan = find(~nan); % Indices

ps(3,:) = pg(3) + swivelAxis(3,:) .* srw;
ps(2,:) = pg(2) + swivelAxis(2,:) .* srw;
ps(1,:) = pg(1) + swivelAxis(1,:) .* srw;

% Ts0 is also used for Ts for wrist-2-goal
Ts0 = reshape([swivelAxis; ys; elbowAxis; ps],3,4,[]); Ts0(4,4,:) = 1;
Ts0(:,:,nan) = NaN;

% Rotate swivel frames for median-fan and explicit
Ts0_sz = size(Ts0,3);
switch phiType
    case {3 4}
        Ts = NaN(4,4,Ts0_sz,phi_sz);       
        for Ts0_num = notnan
            for phi_num = 1: phi_sz
                if isnan(phi(Ts0_num,phi_num)), continue;
                else
                    Ts(:,:,Ts0_num,phi_num) = ...
                        Ts0(:,:,Ts0_num)*trotx(phi(Ts0_num,phi_num));
                end
            end
        end
    otherwise % hand-2-goal or zero
        Ts = Ts0;       
end

% Generate forearm and upper arm frames
Tf = NaN(size(Ts));
Tu = NaN(size(Ts));
sQf = atan2(ers,srw);
sQu = atan2(grs,ers);
for Ts0_num = notnan
    for phi_num = 1: phi_sz
        if isnan(Ts(4,4,Ts0_num,phi_num)), continue;
        else
            Tf(1:3,1:3,Ts0_num,phi_num) = ...
                Ts(1:3,1:3,Ts0_num,phi_num) * rotz(sQf(Ts0_num));
            Tf(1:3,4,Ts0_num,phi_num) = ph(:,Ts0_num);
            Tf(4,:,Ts0_num,phi_num) = [0, 0, 0, 1];
            Tu(:,:,Ts0_num,phi_num) = Ts(:,:,Ts0_num,phi_num) * ...
                rt2tr(rotz(sQu(Ts0_num)), [0; -ers(Ts0_num); 0]);
        end
    end
end

Tf = squeeze(reshape(Tf,[4,4,data_sz]));
Ts = squeeze(reshape(Ts,[4,4,data_sz]));
Tu = squeeze(reshape(Tu,[4,4,data_sz]));
end

%% RTB utility functions (modified)

function R = rotz(t)
    ct = cos(t);
    st = sin(t);
    R = [ct -st 0; st ct 0; 0 0 1];
end

function T = rt2tr(R, t)
if size(R,3) > 1
    Z = zeros(size(R,2),1);
    B = [Z' 1];
    T = zeros(4,4,size(R,3));
    for i=1:size(R,3)
        T(:,:,i) = [R(:,:,i) t(:,i); B];
    end
else
    T = [R t; zeros(1,size(R,2)) 1];
end
end

function S = skew(v)
S = [  0   -v(3)  v(2)
    v(3)  0    -v(1)
    -v(2) v(1)   0];
end

function T = trotx(t)
ct = cos(t);
st = sin(t);
T = [1 0 0 0; 0 ct -st 0; 0 st ct 0; 0 0 0 1];
end