%EDGELIST Return list of edge pixels for region
%
% EG = EDGELIST(IM, SEED) is a list of edge pixels (Nx2) of a region in the
% image IM starting at edge coordinate SEED=[X,Y].  The edgelist has one row per
% edge point coordinate (x,y).  
%
% EG = EDGELIST(IM, SEED, DIRECTION) as above, but the direction of edge
% following is specified.  DIRECTION == 0 (default) means clockwise, non
% zero is counter-clockwise.  Note that direction is with respect to y-axis
% upward, in matrix coordinate frame, not image frame.
%
% [EG,D] = EDGELIST(IM, SEED, DIRECTION) as above but also returns a vector
% of edge segment directions which have values 1 to 8 representing W SW S SE E
% NW N NW respectively.
%
% Notes::
% - Coordinates are given assuming the matrix is an image, so the indices are
%   always in the form (x,y) or (column,row).
% - IM is a binary image where 0 is assumed to be background, non-zero 
%   is an object.
% - SEED must be a point on the edge of the region.
% - The seed point is always the first element of the returned edgelist.
% - 8-direction chain coding can give incorrect results when used with
%   blobs founds using 4-way connectivty.
%
% Reference::
% - METHODS TO ESTIMATE AREAS AND PERIMETERS OF BLOB-LIKE OBJECTS: A COMPARISON
%   Luren Yang, Fritz Albregtsen, Tor Lgnnestad and Per Grgttum
%   IAPR Workshop on Machine Vision Applications Dec. 13-15, 1994, Kawasaki
%
% See also ILABEL.

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

function [e,d] = edgelist(im, P, direction)

    % deal with direction argument
    if nargin == 2
        direction = 0;
    end

    if direction == 0
        neighbours = [1:8]; % neigbours in clockwise direction
    else
        neighbours = [8:-1:1];  % neigbours in counter-clockwise direction
    end

    P = P(:)';
    P0 = P;     % make a note of where we started
    pix0 = im(P(2), P(1));  % color of pixel we start at

    % find an adjacent point outside the blob
    Q = adjacent_point(im, P, pix0);
    if isempty(Q)
        error('no neighbour outside the blob');
    end

    e = P;  % initialize the edge list
    dir = []; % initialize the direction list

    % these are directions of 8-neighbours in a clockwise direction
    dirs = [-1 0; -1 1; 0 1; 1 1; 1 0; 1 -1; 0 -1; -1 -1];

    while 1
        % find which direction is Q
        dQ = Q - P;
        for kq=1:8
            if all(dQ == dirs(kq,:))
                break;
            end
        end


        % now test for directions relative to Q
        for j=neighbours
            % get index of neighbour's direction in range [1,8]
            k = j + kq;
            if k > 8
                k = k - 8;
            end
            dir = [dir; k];

            % compute coordinate of the k'th neighbour
            Nk = P + dirs(k,:);
            try
                if im(Nk(2), Nk(1)) == pix0
                    % if this neighbour is in the blob it is the next edge pixel
                    P = Nk;
                    break;
                end
            end
            Q = Nk;     % the (k-1)th neighbour
        end

        % check if we are back where we started
        if all(P == P0)
            break;
        end

        % keep going, add P to the edgelist
        e = [e; P];
    end
    
    if nargout > 1
        d = dir;
    end
end

function P = adjacent_point(im, seed, pix0)
    % find an adjacent point not in the region
    dirs = [1 0; 0 1; -1 0; 0 -1];
    for d=dirs'
        P = [seed(1)+d(1), seed(2)+d(2)];
        try
            if im(P(2), P(1)) ~= pix0
                return;
            end    
        catch
            % if we get an exception then by definition P is outside the region,
            % since it's off the edge of the image
            return;
        end
    end
    P = [];
end
