%ANTHRODATA Load anthropometric data
%
% Retrieve anthropometric data for a 50th percentile male. Data is
% taken from two sources, surveyed on US Army personnel and matched to
% US Marine Corps personnel.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Anthropometric data is referenced from:
%  Donelson, S.M. & Gordon, C.C. 1996, '1995 Matched Anthropometric
%   Database of U.S. Marine Corps Personnel: Summary Statistics'.
%  Paquette, S.P., Gordon, C.C., Brantley, J.D., Case, H.W. & Gaeta,
%   D.J. 1997, 'A Supplement to the 1995 Matched Anthropometric
%   Database of U.S. Marine Corps Personnel: Summary Statistics'.
%
% Syntax:
%  (1) [note, v1, v2, ..., vN] = anthroData(data1, data2, ..., dataN)
%
% Outputs:
%  note : Short text note about data references
%  v    : The value of the corresponding data measurement
%
% Inputs:
%  data : An anthropometric data keyword. This may be:
%          erw : Length of forearm
%          gre : Length of upper arm
%          pb  : Centre point of head
%          pg  : Centre point of shoulders
%          pn  : Bottom end point of neck
%          pt  : Top end point of torso
%          rb  : Radius of head
%          re  : Radius of elbow
%          ru  : Radius of upper arm
%          rw  : Radius of wrist
%          sg  : Breadth, depth, height of shoulders ellipsoid
%          sn  : Radius, radius, length of neck cylinder
%          st  : Breadth, depth, height of torso elliptical cylinder
%          wrh : Length of hand
%
% See also cmdl_arm cmdl_hat HAL

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

function [refnote, varargout] = anthroData(varargin)

refnote = ...
    ['Anthropometric data: ', ...
    '(Donelson & Gordon 1996) ', ...
    '(Paquette et al. 1997)'];

% ...x  is measurement x from Donelson & Gordon
% ...xS is meaurement x from Paquette et al.

% 50th percentile male
acromHeight = 1444.6 /1000; % 2
acromRadiale = 340.7 /1000; % 2S
axArmCirc = 336.6 /1000; % 4
biacrBreadth = 397.4 /1000; % 3S
bidelBreadth = 493.3 /1000; % 8
cervicHeight = 1521.5 /1000; % 20
chestBreadth = 321.7 /1000; % 21
chestDepth = 243.4 /1000; % 24
crotchHeight = 835.9 /1000; % 25
elbowCirc = 277.3 /1000; % 11S
forearmHand = 482.4 /1000; % 33
handLength = 193 /1000; % 37
headLength = 197.3 /1000; % 40
neckCirc = 379.7 /1000; % 48
stature = 1759 /1000; % 60
wristCirc = 174.4 /1000; % 74

data = struct;

data.erw = ...
    forearmHand - handLength - elbowCirc/(2*pi); % Forearm length
data.gre = ...
    acromRadiale - cervicHeight + acromHeight; % Upper arm length
data.pb = ... % Point of head centre
    -[biacrBreadth/2; headLength/2+acromHeight-stature; ...
    chestDepth/2 - headLength/2]; % My own offset
data.pg = [-biacrBreadth/2; 0; 0]; % Point of shoulders
data.pn = [-biacrBreadth/2; 0; 0]; % Point of neck
data.pt = [-biacrBreadth/2; 0; 0]; % Point of torso
data.rb = headLength/2; % Radius of head
data.re = elbowCirc/(2*pi); % Radius of elbow
data.ru = axArmCirc/(2*pi); % Radius of shoulder
data.rw = wristCirc/(2*pi); % Radius of wrist
data.sg = [bidelBreadth/2, (cervicHeight - acromHeight), chestDepth/2];
data.sn = [neckCirc/(2*pi), neckCirc/(2*pi), ...
    -(headLength/2 + acromHeight - stature)]; % Scale vector of neck
data.st = [chestBreadth/2, chestDepth/2, acromHeight-crotchHeight];
data.wrh = handLength; % Hand length


varargout = cell(size(varargin));
for i = 1: length(varargin)
    if isfield(data, varargin{i})
        varargout{i} = getfield(data, varargin{i}); %#ok<GFLD>
    else
        error(pHRIWARE('error','inputValue'));
    end
end