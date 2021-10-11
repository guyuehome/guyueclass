%PNMFILT Pipe image through PNM utility
%
% OUT = PNMFILT(CMD) runs the external program given by the string CMD
% and the output (assumed to be PNM format) is returned as OUT.
%
% OUT = PNMFILT(CMD, IM) pipes the image IM through the external program
% given by the string CMD and the output is returned as OUT.  The external 
% program must accept and return images in PNM format.
%
% Examples::
%       im = pnmfilt('ppmforge -cloud');
%       im = pnmfilt('pnmrotate 30', lena);
%
% Notes::
%  - Provides access to a large number of Unix command line utilities such
%    as ImageMagick and netpbm.
%  - The input image is passed as stdin, the output image is assumed to
%    come from stdout.
%  - MATLAB doesn't support i/o to pipes so the image is written to a
%    temporary file, the command run to another temporary file, and that
%    is read into MATLAB.
%
% See also PGMFILT, IREAD.

% Copyright (C) 1993-2011, by Peter I. Corke
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

function im2 = pnmfilt(cmd, im)

	% MATLAB doesn't support pipes, so it all has to be done via 
	% temp files :-(

    % create temporary output file
	ofile = sprintf('%s.pnm', tempname);

    quiet = '2> /dev/null';

    if nargin < 2
        unix([cmd ' > ' ofile ' ' quiet]);
    else
        % create temporary input file
        ifile = sprintf('%s.pnm', tempname);
        imwrite(im, ifile, 'pnm');
        unix([cmd ' < ' ifile ' > ' ofile ' ' quiet]);
    end

	im2 = idouble( imread(ofile) );

    if nargin < 2
        unix(['/bin/rm ' ofile]);
    else
        unix(['/bin/rm ' ifile ' ' ofile]);
    end
