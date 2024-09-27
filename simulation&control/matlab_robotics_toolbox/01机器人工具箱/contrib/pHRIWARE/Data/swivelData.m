%SWIVELDATA Create look-up tables of swivel angle median and range
% 
% Will save the file SwivelData.mat to the same location as this
% function. The median and range look-up tables are called phi_med and
% phi_range respectively. Angles are in radians, and non-reachable
% points are NaN. They are made with 1deg resolution, and are nxnxn in
% size, where n may be set, but default 101.
% 
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% Known issues:
%  - The model for the shoulder range of motion seems to give some
%     outlier results, which causes in a large swivel angle range
%  - When interpolating to find a value, when near the surface of the
%     arm's reacahble workspace, a NaN may be returned though it is
%     reachable

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

w = which('wrapToPi');
if isempty(w)
    error(pHRIWARE('error', ...
        ['The MATLAB wrapToPi function could not be detected.', ...
        'Replace it with your own version']));
end

n = 101;

hal = HAL();
AP = hal.AP;
reach = sum([AP{2} AP{3}]);

X = linspace(-reach,reach,n);
Y = linspace(-reach,reach,n);
Z = linspace(-reach,reach,n);
PHI = d2r*(-180:179);

phi_med = zeros(n,n,n,'single');
phi_range = zeros(n,n,n,'single');

Q1 = zeros(length(PHI), 7);
Q2 = zeros(length(PHI), 7);

for x = 1:n
    for y = 1:n
        for z = 1:n
            [~,~,Tu] = h2fsu(AP,[X(x),Y(y),Z(z)]',PHI);
            [Q1(:,1:3), Q2(:,1:3)] = gikine(AP{1},Tu);
            [~, u] = hal.reachable(Q1, Q2);
                   
            valid = logical(~u');
%             v = find(valid);
%             if ~isempty(v)
%                 d = diff([v, v(end)+v(1)]);
%                 if ~all(d==1)
%                     longest = [0 0 0];
%                     count = 0;
%                     for i=1:length(d)
%                         if d(i) == 1
%                             count = count+1;
%                         else
%                             if count > longest(3)
%                                 longest = [i-count i-1 count];
%                             end
%                             count = 0;
%                         end
%                     end
%                     if ~isequal(longest,[0 0 0])
%                         valid = v(longest(1):longest(2));
%                     else
%                         valid = false(size(PHI));
%                     end
%                 else
%                     valid = v;
%                 end
%             end
                
                reachablePhi = PHI(valid);
                
                if ~isempty(reachablePhi)
                    phiM = max(reachablePhi);
                    phim = min(reachablePhi);
                   
                    if phim == PHI(1) && phiM == PHI(end)
                        unreachablePhi = PHI(logical(~valid));
                        phiM = min(unreachablePhi) + 1*d2r + 2*pi;
                        phim = max(unreachablePhi) - 1*d2r;
                    end
                else
                    phiM = NaN;
                    phim = NaN;    
                end
                
            
            phi_med(x,y,z) = (phiM+phim)/2;
            phi_range(x,y,z) = (phiM-phim)/2;
        end
    end
    fprintf('.');
    if ~mod(x,10), fprintf('\n'); end
end

phi_med = wrapToPi(phi_med);

dir = fileparts(which('swivelData.m'));
save([dir,'\SwivelData.mat'],'phi_med','phi_range');
fprintf('saved!\n');
