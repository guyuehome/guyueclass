% Copyright (C) 1995-2009, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

fprintf('** building MEX files for MVTB\n');
pth = which('imorph.m');
pth = fileparts(pth);
cd( fullfile(pth, 'mex') );

mex CFLAGS=-std=c99 closest.c
mex CFLAGS=-std=c99 fhist.c
mex CFLAGS=-std=c99 hist2d.c
mex CFLAGS=-std=c99 ilabel.c
mex CFLAGS=-std=c99 imatch.c
mex CFLAGS=-std=c99 imorph.c
mex CFLAGS=-std=c99 irank.c
mex CFLAGS=-std=c99 ivar.c
mex CFLAGS=-std=c99 iwindow.c
mex CFLAGS=-std=c99 stereo_match.c
